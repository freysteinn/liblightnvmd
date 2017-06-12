# liblightnvmd

Welcome to the liblightnvmd project. This project objective is to expose
Open-Channel SSD enabled disks to userspace by combining NVMeDirect and
Liblightnvm.

## Motivation

This project is part of a masters thesis where the motivation is to compare the
performance of Open-Channel SSD when the device is exposed directly in userspace
to the traditional lightnvm kernel module.

## Components

The project consists of the four components listed bellow:

- nvmedirect

   The nvmedirect directory contains a fork of the [nvmedirect](https://github.com/nvmedirect/nvmedirect) project's
   kernel module which has been modified to work with an Open-Channel SSD.
  
- liblightnvmd

   The liblightnvm directory contains a fork of the [liblighnvm](https://github.com/OpenChannelSSD/liblightnvm) project. The
   fork adds a nvmedirect backend to the library, but it also contains some
   minor changes to disable the other backends because they can not function at
   the same time.
   
- fox

  The [fox](https://github.com/DFC-OpenSource/fox) program is a tool for testing I/O parallelism in Open-Channel SSD
  disks written by Ivan L. Picoli, and is included here for completeness.
  All of our benchmarks have been evaluated using the fox program.

- helpers

  This directory contains helper programs such as a cleaner in the
  event of an unclean shutdown of a liblightnvmd program.

## Installation

The following steps have to be taken to run the fox program with liblightnvmd.

1. Compile and insert the nvmedirect kernel module.

```
cd nvmedirect
make
sudo insmod modules/nvmed.ko
cd ..
```

Keep in mind that the nvmedirect kernel module relies on the nvme module being
already leaded.

Note: If you are using the kernel module with an uninstalled kernel, then you
should change the KERNELDIR variable in the nvmedirect/modules/Makefile to point
to your kernel source directory.

2. Compile the liblightnvmd library.

```
cd liblightnvmd
make
cd ..
```

3. Compile the fox benchmark program.

If you don't want to install the liblightnvmd library, you should change the fox/Makefile line

```
LLNVM = /usr/local/lib/liblightnvm.a
```

to

```
LLNVM = ../liblighnvmd/build/liblightnvm.a
```

To compile fox run

```
cd fox
make
```

4. Running fox

You can now run fox using it's standard parameters. Please look at the fox
documentation for more.

```
sudo ./fox run -d /dev/nvme0n1 -j 8 -c 8 -l 4 -b 2 -p 128 -m 3 -r 100 -v 8 -e 2 -o
```

## License

This project consists of modules which either use the GPL license or the BSD
license. Please lookup the respective license in each folder or lookup the
original source it was forked from.
