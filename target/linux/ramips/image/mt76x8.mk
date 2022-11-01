include ./common-tp-link.mk

DEFAULT_SOC := mt7628an

define Device/WMD-7688A-12816
  IMAGE_SIZE := 14912k
  DEVICE_VARIANT := 16M
  BLOCKSIZE := 64k
  DEVICE_MODEL := WMD-7688A-12816
  DEVICE_PACKAGES := kmod-mt76x2 kmod-usb2 kmod-usb-ohci luci \
	drv_regopt reg \
	kmod-mt76 kmod-lib80211 \
	kmod-i2c-mt7628 kmod-sdhci-mt7620 kmod-usb-storage \
	quectel mt76x8_base
  IMAGES += factory.bin
  IMAGE/factory.bin := $$(sysupgrade_bin) | pad-to $$$$(BLOCKSIZE) | \
	check-size
endef
TARGET_DEVICES += WMD-7688A-12816

define Device/WMD-7688A-12832
  IMAGE_SIZE := 32448k
  DEVICE_VARIANT := 32M
  BLOCKSIZE := 64k
  DEVICE_MODEL := WMD-7688A-12832
  DEVICE_PACKAGES := kmod-mt76x2 kmod-usb2 kmod-usb-ohci luci \
	drv_regopt reg \
	kmod-mt76 kmod-lib80211 \
	kmod-i2c-mt7628 kmod-sdhci-mt7620 kmod-usb-storage \
	quectel mt76x8_base
  IMAGES += factory.bin
  IMAGE/factory.bin := $$(sysupgrade_bin) | pad-to $$$$(BLOCKSIZE) | \
	check-size
endef
TARGET_DEVICES += WMD-7688A-12832
