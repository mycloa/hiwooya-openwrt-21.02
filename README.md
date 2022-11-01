##### Official Website: www.hi-wooya.com
##### QQ: 317312379

### ethmode command
| command |   status   |  
|---|---|
| ethmode l | only Port0,LAN |
| ethmode w | only Port0,WAN |
| ethmode wllll | Port0-Port4, Port0 is WAN,Port1-4 is LAN |
| ethmode lllll | Port0-Port4,all is LAN |


### wifimode command
| command |   status   |  
|---|---|
| wifimode ap | only AP,LAN |
| wifimode sta | only STA(apcli0),WAN |
| wifimode apsta | AP+STA,AP is LAN,STA is WAN |

### how to check if connected to some AP? use ap_client command,check return is ok or no
``` sh
$ ap_client
```
ok is connected
no is not connected

### How to compile?
# 1.install depend
## Ubuntu16.04
$ sudo apt-get update

$ sudo apt-get install git g++ make libncurses5-dev subversion libssl-dev gawk libxml-parser-perl unzip wget python xz-utils vim zlibc zlib1g zlib1g-dev openjdk-8-jdk build-essential ccache gettext xsltproc 
## Macos
note: install brew and Xcode command line tools

$brew install coreutils findutils gawk gnu-getopt gnu-tar grep wget quilt xz

note: gnu-getopt is keg-only, so force linking it:brew ln gnu-getopt --force

# 2.download the source use git
$ git clone https://gitee.com/hi-wooya/openwrt-21.02.git
# 3.update the feeds
$ cd openwrt-21.02
$ ./scripts/feeds update -a

$ ./scripts/feeds install -a
# 4.config
$ make menuconfig
select the target:

Target System(MediaTek Ralink MIPS) --->

Subtarget(MT76x8 based board) --->

Target Profile(WMD-7688A-12816) --->

## note
WMD-7688A-12816:		16MB FLASH + 128MB RAM

WMD-7688A-12832 32M:	32MB FLASH + 128MB RAM

# 5.make
$ make -j4

# 6.image
the binary image name like this in bin/targets/ramips/mt76x8:
openwrt-ramips-mt76x8-WMDxxx-xxx-xxx-squashfs-sysupgrade.bin
