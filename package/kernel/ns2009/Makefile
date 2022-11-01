#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=ns2009_ts
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/ns2009_ts
  SUBMENU:=Other modules
  TITLE:=NS2009 based touchscreens
  DEPENDS:=+kmod-i2c-core
  FILES:=$(PKG_BUILD_DIR)/ns2009_ts.ko
  AUTOLOAD:=$(call AutoLoad,30,ns2009_ts,1)
endef

define KernelPackage/ns2009_ts/description
 This is a replacement for the following in-kernel drivers:
  To compile this driver as a module, choose M here: the
  module will be called tsc2007.
endef

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)"

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(KERNEL_MAKE_FLAGS) \
		M="$(PKG_BUILD_DIR)" \
		EXTRA_CFLAGS="$(BUILDFLAGS)" \
		modules
endef

$(eval $(call KernelPackage,ns2009_ts))
