/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2007 - 2026 Intel Corporation. */

#include "igb.h"
#include "e1000_82575.h"
#include "e1000_hw.h"
#ifdef IGB_HWMON
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/hwmon.h>
#include <linux/pci.h>
#include "tmp102.h"

#define BOARD2409_MAX_SENSORS 2


/* hwmon callback functions */
static ssize_t igb_hwmon_show_location(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct hwmon_attr *igb_attr = container_of(attr, struct hwmon_attr,
						     dev_attr);
        struct igb_adapter *adapter = container_of(igb_attr->hw, struct igb_adapter, hw);

	int i = igb_attr->name[4] - 0x30;

	if(i == 0) {
		if(adapter->i2c_tmpocxo) return sprintf(buf, "ocxo (ID:%i)\n", adapter->dipsw);
	} else if(i == 1) {
		if(adapter->i2c_tmp) return sprintf(buf, "board (ID:%i)\n", i);
	}

	return -EPERM;
}

static ssize_t igb_hwmon_show_temp(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct hwmon_attr *igb_attr = container_of(attr, struct hwmon_attr,
						     dev_attr);

        struct igb_adapter *adapter = container_of(igb_attr->hw, struct igb_adapter, hw);

	int value = -2;
	int i = igb_attr->name[4] - 0x30;

	/* tmp102 value in 2 * millidegree */
	if(i == 0) {
		if(adapter->i2c_tmpocxo) value = tmp102_read(adapter->i2c_tmpocxo);
	} else if(i == 1) {
		if(adapter->i2c_tmp) value = tmp102_read(adapter->i2c_tmp);
	}

	value /= 2;

	return sprintf(buf, "%i\n", value);
}

static ssize_t igb_hwmon_show_cautionthresh(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct hwmon_attr *igb_attr = container_of(attr, struct hwmon_attr,
						     dev_attr);
        //struct igb_adapter *adapter = container_of(igb_attr->hw, struct igb_adapter, hw);

	unsigned int value = igb_attr->sensor->caution_thresh;
//	if((res = tmp102_init(adapter->i2c_tmpocxo, lmkocxotemp - 5, lmkocxotemp)) == 0) {
//	ocxoready = tmp102_read_alert(adapter->i2c_tmpocxo) ? 1 : 0;

	/* display millidegree */
	value = 68;
	value *= 1000;

	return sprintf(buf, "%u\n", value);
}

static ssize_t igb_hwmon_show_maxopthresh(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct hwmon_attr *igb_attr = container_of(attr, struct hwmon_attr,
						     dev_attr);
	unsigned int value = igb_attr->sensor->max_op_thresh;

	/* display millidegree */
	value = 75;
	value *= 1000;

	return sprintf(buf, "%u\n", value);
}

static ssize_t igb_hwmon_show_dpll(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct hwmon_attr *igb_attr = container_of(attr, struct hwmon_attr,
						     dev_attr);

        struct igb_adapter *adapter = container_of(igb_attr->hw, struct igb_adapter, hw);

	int value = 0;
	int i = igb_attr->name[5] - 0x30; //dpll_%u_state

/*
convert flags to value:
        adapter->dpll_flags; //DPLL_FLAGS_LOPL_DPLL DPLL_FLAGS_LOFL_DPLL DPLL_FLAGS_HLDOVR
eec_holdover_value         4
eec_locked_ho_value        3
eec_locked_value           2
eec_freerun_value          1
eec_invalid_value          0
*/
	if(i == 0) {
		if(adapter->i2c_lmk05318b) {
			value = 1;
			if((adapter->dpll_flags & DPLL_FLAGS_LOFL_DPLL) == 0) value = 3;
			if((adapter->dpll_flags & DPLL_FLAGS_LOPL_DPLL) == 0) value = 2;
			if((adapter->dpll_flags & DPLL_FLAGS_HLDOVR) ) value = 4;
		}
	} else if(i == 1) {
		if(adapter->i2c_tmp) value = tmp102_read(adapter->i2c_tmp);
	}

	return sprintf(buf, "%i\n", value);
}


/* igb_add_hwmon_attr - Create hwmon attr table for a hwmon sysfs file.
 * @ adapter: pointer to the adapter structure
 * @ offset: offset in the eeprom sensor data table
 * @ type: type of sensor data to display
 *
 * For each file we want in hwmon's sysfs interface we need a device_attribute
 * This is included in our hwmon_attr struct that contains the references to
 * the data structures we need to get the data to display.
 */
static int igb_add_hwmon_attr(struct igb_adapter *adapter,
				unsigned int offset, int type) {
	int rc;
	unsigned int n_attr;
	struct hwmon_attr *igb_attr;

	switch (offset) {
	case 0:
		if(adapter->i2c_tmpocxo) break;
		return 0;
	case 1:
		if(adapter->i2c_tmp) break;
		return 0;
	default:
		rc = -EPERM;
		return rc;
	}

	n_attr = adapter->igb_hwmon_buff2.n_hwmon;
	igb_attr = &adapter->igb_hwmon_buff2.hwmon_list[n_attr];

	switch (type) {
	case IGB_HWMON_TYPE_LOC:
		igb_attr->dev_attr.show = igb_hwmon_show_location;
		snprintf(igb_attr->name, sizeof(igb_attr->name),
			 "temp%u_label", offset);
//		netdev_err(adapter->netdev, "sysfs hwmon TMP102 igb_add_hwmon_attr LOC");
		break;
	case IGB_HWMON_TYPE_TEMP:
		igb_attr->dev_attr.show = igb_hwmon_show_temp;
		snprintf(igb_attr->name, sizeof(igb_attr->name),
			 "temp%u_input", offset);
//		netdev_err(adapter->netdev, "sysfs hwmon TMP102 igb_add_hwmon_attr TEMP");
		break;
	case IGB_HWMON_TYPE_CAUTION:
		igb_attr->dev_attr.show = igb_hwmon_show_cautionthresh;
		snprintf(igb_attr->name, sizeof(igb_attr->name),
			 "temp%u_max", offset);
//		netdev_err(adapter->netdev, "sysfs hwmon TMP102 igb_add_hwmon_attr CAUTION");
		break;
	case IGB_HWMON_TYPE_MAX:
		igb_attr->dev_attr.show = igb_hwmon_show_maxopthresh;
		snprintf(igb_attr->name, sizeof(igb_attr->name),
			 "temp%u_crit", offset);
//		netdev_err(adapter->netdev, "sysfs hwmon TMP102 igb_add_hwmon_attr MAX");
		break;
	case IGB_HWMON_TYPE_DPLL:
		igb_attr->dev_attr.show = igb_hwmon_show_dpll;
		snprintf(igb_attr->name, sizeof(igb_attr->name),
			 "dpll_%u_state", offset);
//		netdev_err(adapter->netdev, "sysfs hwmon TMP102 igb_add_hwmon_attr DPLL");
		break;
	default:
		rc = -EPERM;
		return rc;
	}

	/* These always the same regardless of type */
	igb_attr->sensor =
		&adapter->hw.mac.thermal_sensor_data.sensor[offset];
	igb_attr->hw = &adapter->hw;
	igb_attr->dev_attr.store = NULL;
	igb_attr->dev_attr.attr.mode = 0444;
	igb_attr->dev_attr.attr.name = igb_attr->name;
	sysfs_attr_init(&igb_attr->dev_attr.attr);
	rc = device_create_file(&adapter->pdev->dev,
				&igb_attr->dev_attr);
	if (rc == 0)
		++adapter->igb_hwmon_buff2.n_hwmon;


	return rc;
}

static void igb_sysfs_del_adapter(struct igb_adapter *adapter)
{
	int i;

	if (adapter == NULL)
		return;

	for (i = 0; i < adapter->igb_hwmon_buff2.n_hwmon; i++) {
		device_remove_file(&adapter->pdev->dev,
			   &adapter->igb_hwmon_buff2.hwmon_list[i].dev_attr);
	}

	kfree(adapter->igb_hwmon_buff2.hwmon_list);

	if (adapter->igb_hwmon_buff2.device)
		hwmon_device_unregister(adapter->igb_hwmon_buff2.device);

}

static ssize_t lmk_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct igb_adapter *adapter = netdev_priv(pci_get_drvdata(to_pci_dev(dev)));

	if (!adapter->i2c_lmk05318b)
		return -ENODEV;
	return sprintf(buf, "R13=%02x R14=%02x R128=%02x R167=%02x R251=%02x R411=%02x\n",
		       adapter->lmk_status[0], adapter->lmk_status[1], adapter->lmk_status[2],
		       adapter->lmk_status[3], adapter->lmk_status[4], adapter->lmk_status[5]);
}
static DEVICE_ATTR_RO(lmk_status);

static ssize_t ocxo_ready_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct igb_adapter *adapter = netdev_priv(pci_get_drvdata(to_pci_dev(dev)));

	if (!adapter->i2c_tmpocxo)
		return -ENODEV;
	return sprintf(buf, "%d\n", adapter->ocxo_ready);
}
static DEVICE_ATTR_RO(ocxo_ready);

static ssize_t phc_align_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct igb_adapter *adapter = netdev_priv(pci_get_drvdata(to_pci_dev(dev)));

	if (!adapter->i2c_lmk05318b)
		return -ENODEV;
	return sprintf(buf, "%s err=%lld lat=%lld steps=%u\n", igb_phc_align_state(adapter),
		       adapter->phc.err, adapter->phc.lat, adapter->phc.steps);
}
static DEVICE_ATTR_RO(phc_align);

static ssize_t pps_align_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct igb_adapter *adapter = netdev_priv(pci_get_drvdata(to_pci_dev(dev)));

	if (!adapter->i2c_lmk05318b)
		return -ENODEV;
	return sprintf(buf, "%s err=%lld reg=0x%llx writes=%u wide=%d scale=%u\n", igb_pps_align_state(adapter),
		       adapter->pps.err, adapter->pps.reg, adapter->pps.writes, adapter->pps.wide,
		       adapter->pps.scale_ppb);
}
static DEVICE_ATTR_RO(pps_align);

static ssize_t ref_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct igb_adapter *adapter = netdev_priv(pci_get_drvdata(to_pci_dev(dev)));

	if (!adapter->i2c_lmk05318b)
		return -ENODEV;
	return sprintf(buf, "%s link=%d time=%d held=%d\n", igb_ref_state(adapter),
		       adapter->ref.latched, adapter->ref.valid, adapter->ref.dpll_held);
}
static DEVICE_ATTR_RO(ref_state);

/* I210 time sync registers, for the day the captures stop again. */
static ssize_t ts_regs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct igb_adapter *adapter = netdev_priv(pci_get_drvdata(to_pci_dev(dev)));
	struct e1000_hw *hw = &adapter->hw;

	return sprintf(buf, "CTRL=%08x CTRL_EXT=%08x TSSDP=%08x TSAUXC=%08x TSIM=%08x TSICR=%08x TIMINCA=%08x AUX0=%08x.%08x AUX1=%08x.%08x\n",
		       E1000_READ_REG(hw, E1000_CTRL), E1000_READ_REG(hw, E1000_CTRL_EXT),
		       E1000_READ_REG(hw, E1000_TSSDP), E1000_READ_REG(hw, E1000_TSAUXC),
		       E1000_READ_REG(hw, E1000_TSIM), E1000_READ_REG(hw, E1000_TSICR),
		       E1000_READ_REG(hw, E1000_TIMINCA),
		       E1000_READ_REG(hw, E1000_AUXSTMPH0), E1000_READ_REG(hw, E1000_AUXSTMPL0),
		       E1000_READ_REG(hw, E1000_AUXSTMPH1), E1000_READ_REG(hw, E1000_AUXSTMPL1));
}
static DEVICE_ATTR_RO(ts_regs);

/* called from igb_main.c */
void igb_sysfs_exit2(struct igb_adapter *adapter)
{
	device_remove_file(&adapter->pdev->dev, &dev_attr_ts_regs);
	device_remove_file(&adapter->pdev->dev, &dev_attr_ref_state);
	device_remove_file(&adapter->pdev->dev, &dev_attr_pps_align);
	device_remove_file(&adapter->pdev->dev, &dev_attr_phc_align);
	device_remove_file(&adapter->pdev->dev, &dev_attr_ocxo_ready);
	device_remove_file(&adapter->pdev->dev, &dev_attr_lmk_status);
	igb_sysfs_del_adapter(adapter);
}

/* called from igb_main.c */
int igb_sysfs_init2(struct igb_adapter *adapter)
{
	struct hwmon_buff *igb_hwmon = &adapter->igb_hwmon_buff2;
	unsigned int i;
	int n_attrs;
	int rc = 0;


	/* If this method isn't defined we don't support thermals */
//	if (adapter->hw.mac.ops.init_thermal_sensor_thresh == NULL)
//		goto exit;

	/* Don't create thermal hwmon interface if no sensors present */
		if (!adapter->i2c_tmpocxo)
			goto exit;

	/* Allocation space for max attributes
	 * max num sensors * values (loc, temp, max, caution)
	 */
	n_attrs = BOARD2409_MAX_SENSORS * 5;
	igb_hwmon->hwmon_list = kcalloc(n_attrs, sizeof(struct hwmon_attr),
					  GFP_KERNEL);
	if (!igb_hwmon->hwmon_list) {
		rc = -ENOMEM;
		goto err;
	}

	igb_hwmon->device =
#ifdef HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS
		hwmon_device_register_with_groups(&adapter->pdev->dev,
						  "board2409", NULL, NULL);
#else
		hwmon_device_register(&adapter->pdev->dev);
#endif /* HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS */
	if (IS_ERR(igb_hwmon->device)) {
		rc = PTR_ERR(igb_hwmon->device);
		goto err;
	}

	for (i = 0; i < BOARD2409_MAX_SENSORS; i++) {

		/* Only create hwmon sysfs entries for sensors that have
		 * meaningful data.
		 */
		if(!adapter->i2c_tmpocxo)
			continue;

		/* Bail if any hwmon attr struct fails to initialize */
//		rc = igb_add_hwmon_attr(adapter, i, IGB_HWMON_TYPE_CAUTION);
		rc |= igb_add_hwmon_attr(adapter, i, IGB_HWMON_TYPE_LOC);
		rc |= igb_add_hwmon_attr(adapter, i, IGB_HWMON_TYPE_TEMP);
//		rc |= igb_add_hwmon_attr(adapter, i, IGB_HWMON_TYPE_MAX);
		rc |= igb_add_hwmon_attr(adapter, i, IGB_HWMON_TYPE_DPLL);
		if (rc)
			goto err;
	}
	rc = device_create_file(&adapter->pdev->dev, &dev_attr_lmk_status);
	if (rc)
		goto err;
	rc = device_create_file(&adapter->pdev->dev, &dev_attr_ocxo_ready);
	if (rc)
		goto err;
	rc = device_create_file(&adapter->pdev->dev, &dev_attr_phc_align);
	if (rc)
		goto err;
	rc = device_create_file(&adapter->pdev->dev, &dev_attr_pps_align);
	if (rc)
		goto err;
	rc = device_create_file(&adapter->pdev->dev, &dev_attr_ref_state);
	if (rc)
		goto err;
	rc = device_create_file(&adapter->pdev->dev, &dev_attr_ts_regs);
	if (rc)
		goto err;

	goto exit;

err:
	igb_sysfs_del_adapter(adapter);
exit:
	return rc;
}
#endif /* IGB_HWMON */
