include $(TOPDIR)/rules.mk

# Name and release number of this package
PKG_NAME:=mt7621_base
PKG_RELEASE:=V1

PKG_BUILD_DIR := $(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

# Specify package information for this program.
# The variables defined here should be self explanatory.
# If you are running Kamikaze, delete the DESCRIPTION
# variable below and uncomment the Kamikaze define
# directive for the description below
define Package/$(PKG_NAME)
	SECTION:=utils
	CATEGORY:=Utilities
	DEPENDS:= @(TARGET_ramips_mt7621)
	TITLE:=mt7621_base file
endef


# Uncomment portion below for Kamikaze and delete DESCRIPTION variable above
define Package/$(PKG_NAME)/description
	For MT7621 or MT7628 machine's base_file add some files
endef


define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)/file
	$(CP) ./file/* $(PKG_BUILD_DIR)/file
endef


define Build/Compile/Default
endef


define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/file/wifimode $(1)/bin/wifimode
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/file/ethmode $(1)/bin/ethmode
endef


# This line executes the necessary commands to compile our program.
# The above define directives specify all the information needed, but this
# line calls BuildPackage which in turn actually uses this information to
# build a package.
$(eval $(call BuildPackage,$(PKG_NAME)))
