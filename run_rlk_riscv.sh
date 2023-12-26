#!/bin/bash

LROOT=$PWD
JOBCOUNT=${JOBCOUNT=$(nproc)}
export ARCH=riscv
export CROSS_COMPILE=riscv64-linux-gnu-
export INSTALL_PATH=$LROOT/rootfs_debian_riscv/boot/
export INSTALL_MOD_PATH=$LROOT/rootfs_debian_riscv/
export INSTALL_HDR_PATH=$LROOT/rootfs_debian_riscv/usr/

kernel_build=$PWD/rootfs_debian_riscv/usr/src/linux/
rootfs_path=$PWD/rootfs_debian_riscv
rootfs_image=$PWD/rootfs_debian_riscv.ext4

rootfs_size=2048
SMP="-smp 4"


if [ $# -lt 1 ]; then
	echo "Usage: $0 [arg]"
	echo "menuconfig: make menuconfig for kernel"
	echo "build_kernel: build the kernel image."
	echo "build_rootfs: build the rootfs image, need root privilege"
	echo "update_rootfs: update kernel modules for rootfs image, need root privilege."
	echo "run: run debian system."
	echo "run debug: enable gdb debug server."
fi

if [ $# -eq 2 ] && [ $2 == "debug" ]; then
	echo "Enable qemu debug server"
	DBG="-s -S"
	SMP=""
fi

make_kernel_image(){
		echo "start build kernel image..."
		make debian_defconfig
		make -j $JOBCOUNT

		cp arch/riscv/boot/Image kmodules
}

make_menuconfig(){
		echo "start re-config kernel"
		make debian_defconfig
		make menuconfig
		cp .config $PWD/arch/riscv/configs/debian_defconfig 
}

prepare_rootfs(){
		if [ -f $rootfs_image ]; then
			echo "rootfs_debian_riscv.ext4 existing!!!"
			echo "pls run: sudo ./run_debian_riscv.sh update_rootfs."
			echo "or delet the ext4 image and run again!"
			exit 1
		fi

		if [ ! -d $rootfs_path ]; then
			echo "decompressing rootfs..."
			# split -d -b 80m rootfs_debian_riscv.tar.xz -- rootfs_debian_riscv.part 
			cat rootfs_debian_riscv.part0* > rootfs_debian_riscv.tar.xz
			tar -Jxf rootfs_debian_riscv.tar.xz
		fi
}

build_kernel_devel(){
	kernver="$(cat include/config/kernel.release)"
	echo "kernel version: $kernver"

	mkdir -p $kernel_build
	rm rootfs_debian_riscv/lib/modules/$kernver/build
	cp -a include $kernel_build
	cp Makefile .config Module.symvers System.map vmlinux $kernel_build
	mkdir -p $kernel_build/arch/riscv/
	mkdir -p $kernel_build/arch/riscv/kernel/
	mkdir -p $kernel_build/lib/

	cp -a arch/riscv/include $kernel_build/arch/riscv/
	cp -a arch/riscv/Makefile $kernel_build/arch/riscv/

	cp -a arch/riscv/kernel/vdso $kernel_build/arch/riscv/kernel/
	cp -a lib/vdso $kernel_build/lib/

	ln -s /usr/src/linux rootfs_debian_riscv/lib/modules/$kernver/build

	# cp from debian linux-kbuild package
	cp -a $rootfs_path/usr/lib/linux-kbuild-6.5.0-5/scripts $kernel_build
	cp -a $rootfs_path/usr/lib/linux-kbuild-6.5.0-5/tools $kernel_build

	cp  scripts/module.lds $kernel_build/scripts/
}

check_root(){
		if [ "$(id -u)" != "0" ];then
			echo "superuser privileges are required to run"
			echo "sudo ./run_debian_riscv.sh build_rootfs"
			exit 1
		fi
}

update_rootfs(){
		if [ ! -f $rootfs_image ]; then
			echo "rootfs image is not present..., pls run build_rootfs"
		else
			echo "update rootfs ..."

			mkdir -p $rootfs_path
			echo "mount ext4 image into rootfs_debian_riscv"
			mount -t ext4 $rootfs_image $rootfs_path -o loop

			make install
			make modules_install -j $JOBCOUNT
			#make headers_install

			build_kernel_devel

			umount $rootfs_path
			chmod 777 $rootfs_image

			rm -rf $rootfs_path
		fi

}

build_rootfs(){
		if [ ! -f $rootfs_image ]; then
			make install
			make modules_install -j $JOBCOUNT
			# make headers_install

			build_kernel_devel

			echo "making image..."
			dd if=/dev/zero of=$rootfs_image bs=1M count=$rootfs_size
			mkfs.ext4 $rootfs_image
			mkdir -p tmpfs
			echo "copy data into rootfs..."
			mount -t ext4 $rootfs_image tmpfs/ -o loop
			cp -af $rootfs_path/* tmpfs/
			umount tmpfs
			chmod 777 $rootfs_image

			rm -rf $rootfs_path
		fi

}

QEMU=qemu-system-riscv64-8
machine_arg="-machine virt,aia=aplic-imsic,aia-guests=4  -cpu rv64 -m 1024 $SMP -nographic"
kernel_img="-bios opensbi-riscv64-generic-fw_dynamic.bin -kernel arch/riscv/boot/Image"

# kernel args
rootfs_arg="root=/dev/vda rootfstype=ext4 rw"
kernel_arg="nokaslr earlycon=sbi"
crash_arg=""
dyn_arg=""
debug_arg="loglevel=8 sched_debug"
kernel_args="$rootfs_arg $kernel_arg $debug_arg $crash_arg $debug_arg"

# devices
device_arg+=" -device x-riscv-iommu-pci,addr=1.0"
device_arg+=" -drive if=none,file=$rootfs_image,id=hd0 -device virtio-blk-pci,drive=hd0,disable-legacy=on,disable-modern=off,iommu_platform=on,ats=on" 
device_arg+=" -device virtio-net-device,netdev=usernet -netdev user,id=usernet,hostfwd=tcp:127.0.0.1:5555-:22"
device_arg+=" --fsdev local,id=kmod_dev,path=./kmodules,security_model=none -device virtio-9p-device,fsdev=kmod_dev,mount_tag=kmod_mount -device igb"

guest_arg=" -drive file=guest.ext4,read-only=off,id=nvme1 -device nvme,serial=87654321,drive=nvme1,addr=4.0"

# add "nokaslr" into kernel command line or disable CONFIG_RANDOMIZE_BASE
run_qemu_debian(){
		if [ ! -f $LROOT/kmodules ]; then
			mkdir -p $LROOT/kmodules
		fi

		cmd="$QEMU $DBG $machine_arg $kernel_img -append \"$kernel_args\" $device_arg $guest_arg"
		echo "running:"
		echo $cmd
		eval $cmd
}

case $1 in
	build_kernel)
		make_kernel_image
		#prepare_rootfs
		#build_rootfs
		;;
	
	menuconfig)
		make_menuconfig
		;;

	build_rootfs)
		#make_kernel_image
		check_root
		prepare_rootfs
		build_rootfs
		;;

	update_rootfs)
		update_rootfs
		;;
	run)

		if [ ! -f $LROOT/arch/riscv/boot/Image ]; then
			echo "canot find kernel image, pls run build_kernel command firstly!!"
			exit 1
		fi

		if [ ! -f $rootfs_image ]; then
			echo "canot find rootfs image, pls run build_rootfs command firstly!!"
			exit 1
		fi

		#prepare_rootfs
		#build_rootfs
		run_qemu_debian
		;;
esac

