#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#error " ==================================================="
#error " ==  kernel too old for port monitoring feature   =="
#error " ==  [requires 3.10 or newer]                     =="
#error " ==================================================="
#pragma GCC diagnostic error "-Wfatal-errors"
#error
#endif

#include <linux/proc_fs.h>

struct monitorRecord
{
  unsigned txrx;
  unsigned len;
  uint8_t  data[512];
};

#define MaxRecords 32  // must be power of 2

#define SpaceRequired(b) (70*(b/16))

#define FullRecordDispLen SpaceRequired(512)

// Displaying a full 512 byte record takes 'FullRecordDispLen' If you
// change the display format (showLine/showRecord below) you must
// update the SpaceRequired() macro above.

struct portMonitor
{
  char path[128];
  int port;
  unsigned rdindex;
  unsigned wrindex;
  spinlock_t lock;
  wait_queue_head_t wqueue;
  struct monitorRecord record[MaxRecords];
  char outbuf[FullRecordDispLen];
  unsigned outbuf_len;
  unsigned outbuf_pos;
};

typedef struct portMonitor portMonitor;

#define PortMonitorIncrementIndex(index)  index = (index+1) & (MaxRecords-1)

static void portMonitorNext(portMonitor *monp)
{
  PortMonitorIncrementIndex(monp->wrindex);
  monp->record[monp->wrindex].len = 0;
  if (monp->rdindex == monp->wrindex)
    PortMonitorIncrementIndex(monp->rdindex);
  wake_up_interruptible(&monp->wqueue);
}

static void portMonitorQueueBytes(portMonitor *monp, unsigned txrx, const uint8_t *data, unsigned len, int flush)
{
  struct monitorRecord *wr;

  if (polling_mode)
    spin_lock_bh(&monp->lock);  // we're called from tasklet
  else
    spin_lock(&monp->lock);     // we're called from bh
  
  wr = &monp->record[monp->wrindex];
  if (len)
    {
      if (wr->len && ((wr->txrx != txrx) || ((wr->len + len) > sizeof wr->data)))
        {
          portMonitorNext(monp);
          wr = &monp->record[monp->wrindex];
        }
      wr->txrx = txrx;
      memcpy(wr->data+wr->len, data, len);
      wr->len += len;
    }

  if (flush)
    portMonitorNext(monp);

  if (polling_mode)
    spin_unlock_bh(&monp->lock);  // we're called from tasklet
  else
    spin_unlock(&monp->lock);     // we're called from bh
}

static unsigned portMonitorShowLine(char *buf, char *prefix, uint8_t *data, unsigned count)
{
  char *p = buf;
  unsigned i;

  p += sprintf(p, "%s ", prefix);
  for (i=0; i<count; ++i)
    p += sprintf(p, " %02x",data[i]);
  for (; i<16; ++i)
    p += sprintf(p,"   ");
  p += sprintf(p, "  ");
  for (i=0; i<count; ++i)
    p += sprintf(p, "%c",data[i]>=' ' && data[i]<='~' ? data[i] : '.');
  p += sprintf(p,"\n");
  return p-buf;
}

static unsigned portMonitorShowRecord(char *buf, struct monitorRecord *r)
{
  char *p = buf;
  unsigned len = r->len;
  uint8_t *d = r->data;
  char *prefix = r->txrx ? "tx" : "rx";

  while (len)
    {
      unsigned count = len > 16 ? 16 : len;
      p += portMonitorShowLine(p,prefix,d,count);
      prefix = "  ";
      d += count;
      len -= count;
    }
  
  return p-buf;
}

static int portMonitor_proc_open(struct inode *inode, struct file *filep)
{
  portMonitor *monp = filep->private_data = PDE_DATA(inode);
  spin_lock_bh(&monp->lock);
  // discard old data
  monp->rdindex = monp->wrindex;
  spin_unlock_bh(&monp->lock);
  return 0;
}

static ssize_t portMonitor_proc_read(struct file *filep, char *buf, size_t count, loff_t *offp ) 
{
  portMonitor *monp = (portMonitor*)(filep->private_data);
  int len = 0;

  if (monp->outbuf_len > monp->outbuf_pos)
    {
      // there's truncated output left over from previous call
      len = monp->outbuf_len - monp->outbuf_pos;
      if (len > count)
        len = count;
      memcpy(buf, &monp->outbuf[monp->outbuf_pos], len);
      monp->outbuf_pos += len;
      return len;
    }

  while (len == 0)
    {
      while (monp->rdindex == monp->wrindex)
        if (wait_event_interruptible(monp->wqueue, monp->rdindex != monp->wrindex))
          return -ERESTARTSYS;

      while ((monp->rdindex != monp->wrindex) && (len < count))
        {
          int n;
          struct monitorRecord r, *s;

          // get one record from circular queue
          spin_lock_bh(&monp->lock);
          s = &monp->record[monp->rdindex];
          r.txrx = s->txrx;
          r.len = s->len;
          memcpy(r.data, s->data, r.len);
          PortMonitorIncrementIndex(monp->rdindex);
          spin_unlock_bh(&monp->lock);

          // format output in our buffer
          n = portMonitorShowRecord(monp->outbuf, &r);
          monp->outbuf_len = n;

          // copy as much as we can to user, leave remainder in buffer
          // for next call
          if (n > (count-len))
            n = (count-len);

          memcpy(buf+len, monp->outbuf, n);
          monp->outbuf_pos = n;
          len += n;
        }
    }

  return len;
}

#define EnableTestWrite 1

#if EnableTestWrite
static ssize_t portMonitor_proc_write(struct file *filep, const char *buf, size_t count, loff_t *offp)
{
  if (count)
    {
      portMonitor *monp = (portMonitor*)(filep->private_data);
      int txrx  = buf[0] == 't' || buf[0] == 'T';
      int flush = buf[0] == 'T' || buf[0] == 'F';
      portMonitorQueueBytes(monp, txrx, buf+1, count-1, flush);
    }
  return count;
}
#define FileMode 0660
#else
#define FileMode 0440
#endif

static struct file_operations portMonitor_proc_fops = 
  {
  open: portMonitor_proc_open,
  read: portMonitor_proc_read,
  #if EnableTestWrite
  write: portMonitor_proc_write,
  #endif
  };

static portMonitor *portMonitorNew(int port)
{
  portMonitor *monp = kmalloc(sizeof *monp, GFP_KERNEL);
  if (!monp)
    return NULL;
  memset(monp, 0, sizeof *monp);
  sprintf(monp->path,"driver/rp2/monitor-ttyRP%d",port);
  spin_lock_init(&monp->lock);
  init_waitqueue_head(&monp->wqueue);
  proc_create_data(monp->path, FileMode, NULL, &portMonitor_proc_fops, monp);
  return monp;
}

static void portMonitorFree(portMonitor *monp)
{
  remove_proc_entry(monp->path, NULL);
  kfree(monp);
}

static void portMonitorInit(void)
{
  proc_mkdir("driver/rp2",NULL);
}

static void portMonitorCleanup(void)
{
  remove_proc_entry("driver/rp2", NULL);
}
