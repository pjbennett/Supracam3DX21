#!/bin/bash


MODNAME="$1"
DEFOPTS="$2"
KVER="$3"

test -z "$MODNAME" && MODNAME="$(basename *.ko .ko)"

test "$MODNAME" = "*" &&
{
    echo "No .ko file found in current directory to install"
    exit 1
}

test -f $MODNAME.ko ||
{
    echo "Module $MODNAME.ko not found in current directory"
    exit 1
}

test -z "$KVER" && KVER="$(uname -r)"

test -n "$KVER" ||
{
    echo "Can't determine kernel vesion for install."
    exit 1
}

rm_from_initramfs()
{
    local Image="$1"
    shift
    local NewImage=$(basename $Image).new$$
    local RootTmp=/tmp/tmp_initrd_root_$$
    local RootCpio=tmp_cpio_$$

    rm -rf $RootTmp
    mkdir $RootTmp

    /usr/lib/dracut/skipcpio $Image > $RootCpio

    echo ">>> initrd image: $Image"

    case "$(file $RootCpio)" in
        *:\ gzip\ compressed\ *)
            echo ">>> found gzip compressed image"
            local CatProg=zcat
            local CompProg="gzip --best"
            ;;
        *:\ XZ\ compressed\ *)
            echo ">>> found XZ compressed image"
            local CatProg=xzcat
            local CompProg="xz -0 --check=crc32"
            ;;
        *)
            echo "Don't know how to handle $(file $RootCpio)" 1>&2
            exit 1
            ;;
    esac

    $CatProg $RootCpio | (cd $RootTmp; cpio --quiet -i --preserve-modification-time --no-absolute-filenames)

    local TotalSize=$(stat -c%s $Image)
    local RootCpioSize=$(stat -c%s $RootCpio)
    local HeaderCpioSize=$(($TotalSize - $RootCpioSize))

    #echo "  total image size: $TotalSize"
    #echo "  header cpio size: $HeaderCpioSize"
    #echo "    root cpio size: $RootCpioSize"
    #echo


    for File in "$@"
    do
        local FilePath
        for FilePath in $(find $RootTmp/lib/modules -type f -name "$File")
        do
	    echo ">>> removing $FilePath"
	    rm -f $FilePath
        done
    done    

    (dd status=none if=$Image bs=$HeaderCpioSize count=1; (cd $RootTmp; find . | cpio --quiet -H newc -o | $CompProg)) > $NewImage

    rm -rf $RootTmp $RootCpio

    cp $Image $Image.old
    mv $NewImage $Image

    echo ">>> Created new $Image"
    echo ">>> Previous saved to $Image.old"
}


echo "Installing $MODNAME module for kernel version $KVER"
echo

test -d /lib/modules/$KVER/kernel/drivers ||
{
    echo "Directory /lib/modules/$KVER/kernel/drivers not found."
    echo "Don't know where to install $MODNAME.ko"
    exit 1
}

# remove old-style config file and init script
INITD=$(find /etc/ -type d -name "init.d")
if [ "$MODNAME" = rp2 ]; then
    rm -f /etc/rp_infinity_express.conf
    test -n "$INITD" && rm -f $(find $INITD -name rp_infinity_express)
elif [ "$MODNAME" = rocket ]; then
    rm -f $(find /etc/ -type l -name "*rocket")
    rm -f /sbin/loadrm2 /etc/ctmmdmfw.rm /etc/ctmmdmld.rm
    test -n "$INITD" && rm -f $(find $INITD -name rocket)
fi

# install kernel module
#
# if there is an $MODNAME.ko installed by the distro, it should be in
# kernel/drivers/tty/serial, so we'll install ours above that to
# make sure modprobe finds it first.  Just to be safe, we'll also
# remove any existing $MODNAME driver module.

rm -f $(find /lib/modules/$KVER/kernel/drivers -name "$MODNAME.ko*")
mkdir -p /lib/modules/$KVER/kernel/drivers/tty
echo "installing $MODNAME kernel module in /lib/modules/$KVER/kernel/drivers/tty"
cp $MODNAME.ko /lib/modules/$KVER/kernel/drivers/tty
echo "updating module dependancies"
/sbin/depmod $KVER || depmod $KVER
echo "done"
echo

echo "Setting up '$MODNAME' module:"
echo

# tell dracut not to include this kernel module in initramfs
if [ -d /etc/dracut.conf.d ]; then
    echo "omit_drivers+=\" $MODNAME \"" > /etc/dracut.conf.d/$MODNAME.conf
elif [ -f /etc/dracut.conf ]; then
    if ! grep -q $MODNAME /etc/dracut.conf; then
        echo "omit_drivers+=\" $MODNAME \"" >> /etc/dracut.conf
    fi
fi

if [ "$REBUILD_INITRAMFS" = y ]; then
    rebuild_initramfs
fi

if [ -x /usr/bin/lsinitrd ]; then
    # check to see if current initramfs contains a conflicting driver
    if lsinitrd -k $KVER | fgrep -q "$MODNAME.ko"; then
        cat <<EOF
########################################################################
 Warning: The initial ramdisk for kernel $KVER
          contains a conflicting $MODNAME driver.  The driver you just
          installed will not be used unless the conflicting driver is
          removed from the ramdisk.

          Do you want to rebuild the ramdisk now to remove the conflicting
          driver?
EOF
        read -p '[y/n] ' Answer
        if [ "$Answer" = y -o "$Answer" = Y ]; then
            InitrdImage=$(expr match "$(lsinitrd | grep '^Image: ')" 'Image: \([^:]*\)')
            rm_from_initramfs $InitrdImage "$MODNAME.ko*"
        fi
    echo
    fi
fi


# make sure module gets loaded -- not needed for newer systems that
# will recognize the PCI ID and automatically load the kernel module,
# but won't hurt anything on those newer systems

if [ -d /etc/modules-load.d ]; then
    if test -f /etc/modules-load.d/$MODNAME.conf; then
        echo "/etc/modules-load.d/$MODNAME.conf already exists"
    else
        echo "creating /etc/modules-load.d/$MODNAME.conf"
        echo "$MODNAME" > /etc/modules-load.d/$MODNAME.conf
    fi
elif [ -f /etc/conf.d/modules ]; then
    if grep -q "^modules=.* $MODNAME" /etc/conf.d/modules; then
        echo "/etc/conf.d/modules already contains $MODNAME"
    else
        echo "appending $MODNAME to /etc/conf.d/modules"
        echo "modules=\"\${modules} $MODNAME\"" >> /etc/conf.d/modules
    fi
elif [ -f /etc/sysconfig/modules ]; then
    if grep -q "modprobe $MODNAME" /etc/sysconfig/modules; then
        echo "/etc/sysconfig/modules already contains $MODNAME"
    else
        echo "appending $MODNAME to /etc/sysconfig/modules"
        echo 'modprobe $MODNAME' >> /etc/sysconfig/modules
    fi
elif [ -f /etc/modules ]; then
    if grep -q "$MODNAME" /etc/modules; then
        echo "/etc/modules already contains $MODNAME"
    else
        echo "appending $MODNAME to /etc/modules"
        echo "$MODNAME" >> /etc/modules
    fi
elif [ -f /etc/rc.modules ]; then
    if grep -q "modprobe $MODNAME" /etc/rc.modules; then
        echo "/etc/rc.modules already contains $MODNAME"
    else
        echo "appending $MODNAME to /etc/rc.modules"
        echo 'modprobe $MODNAME' >> /etc/rc.modules
    fi
else
    if fgrep -q /etc/rc.modules /etc/rc.sysinit 2>/dev/null; then
        echo "found /etc/rc.modules referenced in rc.sysinit: creating /etc/rc.modules"
    else
        cat <<EOF
Didn't find any of known mechanisms used to load modules at startup:

   /etc/modules-load.d/
   /etc/conf.d/modules
   /etc/modules
   /etc/rc.modules

Creating /etc/rc.modules file.  If $MODNAME module does not get loaded
at loaded at startup, see your Linux distribution documentation on how
to configure persistem module loading.

EOF
    fi

    echo "#!/bin/sh" >>/etc/rc.modules
    echo "/sbin/modprobe $MODNAME" >>/etc/rc.modules
    chmod +x /etc/rc.modules
fi    
echo

# figure out where to put default module parameters (if needed)

if [ "$DEFOPTS" != "" ]; then
    if [ -d /etc/modprobe.d ]; then
        if [ -f /etc/modprobe.d/$MODNAME.conf ]; then
            echo "/etc/modprobe.d/$MODNAME.conf already exists"
        else
            echo "creating /etc/modprobe.d/$MODNAME.conf with default options:"
            echo "  $DEFOPTS"
            echo "options $MODNAME $DEFOPTS" > /etc/modprobe.d/$MODNAME.conf
        fi
        cat <<EOF

$MODNAME module parameters are in /etc/modprobe.d/$MODNAME.conf.  See
README.txt for information on setting port interface modes and for
polling and interrupt settings

EOF
    else
        echo "\

WARNING: Don't know where to put module parameters for this
         distribution.  If you need to change polling/interrupt
         settings or port interface modes, consult README.txt for
         module parameter names/values and consult your distribution
         documentation regarding where to set them.

"
    fi
fi
