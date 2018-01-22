#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/power/max17048_battery.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/sort.h>

#define LOW_VOLT_THRESHOLD	(2800000)
#define HIGH_VOLT_THRESHOLD	(4200000)

#define MAX17048_VCELL_MSB	(0x02)
#define MAX17048_VCELL_LSB	(0x03)

#define MAX17048_SOC_MSB	(0x04)
#define MAX17048_SOC_LSB	(0x05)

#define MAX17048_MODE_MSB	(0x06)
#define MAX17048_MODE_LSB	(0x07)

#define MAX17048_VER_MSB	(0x08)
#define MAX17048_VER_LSB	(0x09)

#define MAX17048_RCOMP_MSB	(0x0C)
#define MAX17048_RCOMP_LSB	(0x0D)

#define MAX17048_CMD_MSB	(0xFE)
#define MAX17048_CMD_LSB	(0xFF)

#define MAX17048_DELAY		(1000)
#define MAX17048_BATTERY_FULL	(95)

struct max17048_chip {
    struct i2c_client	            *client;
    struct max17048_platform_data   *pdata;
    struct delayed_work	            work;
    struct device                   *dev;
    struct power_supply	            battery; //表示添加电池供电
    
    int online;                 //State Of Connect
    int vcell;                 //battery voltage 电池电压
    int soc;                  //当前电量百分比
    int chg_state;             // State Of Charge 充电状态   charging, discharge, unknown,full等
    int old_percent;         //上一次的电量百分比

    struct power_supply	            usb; //表示添加usb供电
    struct power_supply	            psy; //表示添加dc供电

    int charger_online;
    int usb_charger_online;
    int battery_status;
    int first_delay_count;
    bool fault;
    bool usb_in;
    bool ta_in;

    struct mutex work_lock;
};


static void charger_update_status(struct max17048_chip *data);
static void battery_update_status(struct max17048_chip *data);

static int max17048_write_reg(struct i2c_client *client, int reg, u8 value)
{
    int ret;

    ret = i2c_smbus_write_byte_data(client, reg, value);

    if (ret < 0)
         dev_err(&client->dev, "%s: err %d\n", __func__, ret);

    return ret;
}

static int max17048_read_reg(struct i2c_client *client, int reg)
{
    int ret;

    ret = i2c_smbus_read_byte_data(client, reg);

    if (ret < 0)
         dev_err(&client->dev, "%s: err %d\n", __func__, ret);

    return ret;
}

static void max17048_reset(struct i2c_client *client)
{
    max17048_write_reg(client, MAX17048_CMD_MSB, 0x54);
    max17048_write_reg(client, MAX17048_CMD_LSB, 0x00);

    return ;
}

static void max17048_get_vcell(struct i2c_client *client) //电池电压
{
    struct max17048_chip *chip = i2c_get_clientdata(client);
    u8 msb = max17048_read_reg(client, MAX17048_VCELL_MSB);
    u8 lsb = max17048_read_reg(client, MAX17048_VCELL_LSB);
    chip->vcell = msb <<8 + lsb;

    printk("\nmax17048_get_soc  cell = %d\n",chip->vcell);
    return ;
}

static void max17048_get_soc(struct i2c_client *client) //电量百份比
{
    struct max17048_chip *chip = i2c_get_clientdata(client);
    u8 msb = max17048_read_reg(client, MAX17048_SOC_MSB);
    chip->soc = msb;
    
    //printk("\nmax17048_get_soc  msb = %d\n",msb);
    return;
}

static void max17048_get_version(struct i2c_client *client)//版本
{
    //struct max17048_chip *chip = i2c_get_clientdata(client);
    u8 msb = max17048_read_reg(client, MAX17048_VER_MSB);
    u8 lsb = max17048_read_reg(client, MAX17048_VER_LSB);
    dev_info(&client->dev, "MAX17048 Fuel-Gauge Ver %d%d\n", msb, lsb);
    return;
}
static int max17048_get_property(struct power_supply *psy, enum power_supply_property psp,  union power_supply_propval *val)//得到电池各种状态
{
    struct max17048_chip *chip = container_of(psy, struct max17048_chip, battery);

    switch (psp) {
        case POWER_SUPPLY_PROP_STATUS:
             val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
             if (gpio_get_value(chip->pdata->chg) == 0) 
             {
                 chip->battery_status = POWER_SUPPLY_STATUS_CHARGING;//正在充电
             } 
             else if (chip->ta_in && gpio_get_value(chip->pdata->chg) == 1) 
             {
                 if (!chip->pdata->feature_flag) 
                 {
                    if (chip->soc >= 99)
                        chip->battery_status = POWER_SUPPLY_STATUS_FULL;//电量大于99就充满了
                    else
                        chip->battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                 } 
                 else 
                 {
                    chip->battery_status = POWER_SUPPLY_STATUS_FULL;
                 }
            } else if (chip->usb_in && gpio_get_value(chip->pdata->chg) == 1) {
                     if (!chip->pdata->feature_flag)
                     {
                        if (chip->soc >= 99)
                            chip->battery_status = POWER_SUPPLY_STATUS_FULL;
                        else
                            chip->battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                     } 
                     else 
                     {
                         chip->battery_status = POWER_SUPPLY_STATUS_FULL;
                     }
            }
            val->intval = chip->battery_status;
            return 0;
        default:
              break;
        }

    switch (psp) 
    {
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
             val->intval = chip->vcell;
             break;
        case POWER_SUPPLY_PROP_CAPACITY:
             val->intval = chip->soc < 0 ? 0 :
                          (chip->soc > 100 ? 100 : chip->soc);
             break;
        case POWER_SUPPLY_PROP_PRESENT:
             val->intval = 1;
             break;
        case POWER_SUPPLY_PROP_CHARGE_NOW:
             val->intval = 0;
             break;
        case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
             val->intval = HIGH_VOLT_THRESHOLD;
             break;
        case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
             val->intval = LOW_VOLT_THRESHOLD;
             break;

        case POWER_SUPPLY_PROP_HEALTH:
             val->intval = POWER_SUPPLY_HEALTH_GOOD;
             if (chip->fault)
                 val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
             break;
        case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
             if (chip->battery_status == POWER_SUPPLY_STATUS_FULL)
                 val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
             else if (chip->soc <= 15)
                 val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;//电量小于15%就报低电量
             else
                 val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;//否则就报正常
             break;
        default:
             return -EINVAL;
    }
    return 0;
}

static void max17048_work(struct work_struct *work)
{
    struct max17048_chip *chip;

    chip = container_of(work, struct max17048_chip, work.work);
  //读取电池的状态信息
    max17048_get_vcell(chip->client);    
    max17048_get_soc(chip->client);

    charger_update_status(chip);//检测充电状态

    battery_update_status(chip);//检测电池状态

    schedule_delayed_work(&chip->work, MAX17048_DELAY);//定时器
}

static enum power_supply_property max17048_battery_props[] = {
    POWER_SUPPLY_PROP_VOLTAGE_NOW,//当前电压
    POWER_SUPPLY_PROP_STATUS, //当前充电状态
    POWER_SUPPLY_PROP_PRESENT,//不清楚
    POWER_SUPPLY_PROP_CAPACITY,//电量百分比
    POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,//电池极限电压，最大值
    POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,//电池极限电压，最小值
    POWER_SUPPLY_PROP_HEALTH,//电池健康状态
    POWER_SUPPLY_PROP_CAPACITY_LEVEL,//电量水平，low或者normal
};

static enum power_supply_property charger_props[] = {
    POWER_SUPPLY_PROP_ONLINE, // //外部供电查看是否存在AC或usb
};

static void charger_update_status(struct max17048_chip *data)//检测充电状态
{
    if (data->usb_in || data->ta_in) {
        if (data->ta_in)
            data->charger_online = 1;  //适配器正在充电
        if (data->usb_in)
            data->usb_charger_online = 1; //USb正在充电
    } else {
        data->charger_online = 0;  //适配器未在充电
        data->usb_charger_online = 0; 
    }
    
    if (data->charger_online == 0 && data->usb_charger_online == 0) {
        data->battery_status = POWER_SUPPLY_STATUS_DISCHARGING; //未充电
    } else {
        if (gpio_get_value(data->pdata->chg) == 0) { //正在充电状态
            data->battery_status = POWER_SUPPLY_STATUS_CHARGING; //正在充电状态
        } else if (data->ta_in && gpio_get_value(data->pdata->chg) == 1) {  //未充电
            if (!data->pdata->feature_flag) {
                if (data->soc >= 99)
                    data->battery_status = POWER_SUPPLY_STATUS_FULL; //充满电
                else
                    data->battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                } else {
                    data->battery_status = POWER_SUPPLY_STATUS_FULL;
                }
            } else if (data->usb_in && gpio_get_value(data->pdata->chg) == 1) { //USB
                if (!data->pdata->feature_flag) {
                    if (data->soc >= 99)
                        data->battery_status = POWER_SUPPLY_STATUS_FULL;
                    else
                        data->battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                    } else {
                        data->battery_status = POWER_SUPPLY_STATUS_FULL;
                    }
            }
    }
    pr_debug("chg: %d\n", gpio_get_value(data->pdata->chg));
    pr_debug("dc: %d\n",  gpio_get_value(data->pdata->dok));
    pr_debug("flt: %d\n", gpio_get_value(data->pdata->flt));
}

static void battery_update_status(struct max17048_chip *data)//检测电池状态
{
    int temp = 0;
    static int temp_last;
    bool changed_flag;

    changed_flag = false;
    mutex_lock(&data->work_lock);
    
    if (!data->pdata->feature_flag) {
        max17048_get_vcell(data->client);
        if (temp_last == 0) {
            data->vcell = temp;
            temp_last = temp;
        }
        if (data->charger_online == 0 && temp_last != 0) {//DC充电状态
            temp_last = temp;
            data->vcell = temp;
        }
        if (data->charger_online == 1 || data->usb_charger_online == 1) {//USB充电状态和DC充电状态
            data->vcell = temp;
            temp_last = temp;
        }
        max17048_get_soc(data->client); //计算电量百分比
        
        if (data->soc != data->old_percent) {//电池电压有变化
            data->old_percent = data->soc;
            changed_flag = true;
        }
        if (changed_flag) { //如果有变化
            changed_flag = false;
            power_supply_changed(&data->battery); //告诉android端 电池电量改变了
        }
        
        if (data->first_delay_count < 200) {
            data->first_delay_count = data->first_delay_count + 1 ;
            power_supply_changed(&data->battery);
        }
    }
    mutex_unlock(&data->work_lock);
}

static irqreturn_t max8903_dcin(int irq, void *_data)
{
    struct max17048_chip *data = _data;
    struct max17048_platform_data *pdata = data->pdata;
    bool ta_in;

    ta_in = gpio_get_value(pdata->dok) ? false : true;

    if (ta_in == data->ta_in)
        return IRQ_HANDLED;

    data->ta_in = ta_in;
    pr_info("TA(DC-IN) Charger %s.\n", ta_in ?
                    "Connected" : "Disconnected");
    charger_update_status(data);
    battery_update_status(data);
    power_supply_changed(&data->psy);
    power_supply_changed(&data->battery);
    return IRQ_HANDLED;
}
static irqreturn_t max8903_usbin(int irq, void *_data)
{
    struct max17048_chip *data = _data;
    struct max17048_platform_data *pdata = data->pdata;
    bool usb_in;
    usb_in = gpio_get_value(pdata->uok) ? false : true;
    if (usb_in == data->usb_in)
            return IRQ_HANDLED;

    data->usb_in = usb_in;
    charger_update_status(data);
    battery_update_status(data);
    pr_info("USB Charger %s.\n", usb_in ?
                    "Connected" : "Disconnected");
    power_supply_changed(&data->battery);
    power_supply_changed(&data->usb);
    return IRQ_HANDLED;
}

static irqreturn_t max8903_fault(int irq, void *_data)
{
    struct max17048_chip *data = _data;
    struct max17048_platform_data *pdata = data->pdata;
    bool fault;

    fault = gpio_get_value(pdata->flt) ? false : true;

    if (fault == data->fault)
        return IRQ_HANDLED;

    data->fault = fault;

    if (fault)
        dev_err(data->dev, "Charger suffers a fault and stops.\n");
    else
        dev_err(data->dev, "Charger recovered from a fault.\n");
    charger_update_status(data);
    battery_update_status(data);
    power_supply_changed(&data->psy);
    power_supply_changed(&data->battery);
    power_supply_changed(&data->usb);
    return IRQ_HANDLED;
}

static irqreturn_t max8903_chg(int irq, void *_data)
{
    struct max17048_chip *data = _data;
    struct max17048_platform_data *pdata = data->pdata;
    int chg_state;

    chg_state = gpio_get_value(pdata->chg) ? false : true;

    if (chg_state == data->chg_state)
            return IRQ_HANDLED;

    data->chg_state = chg_state;
    charger_update_status(data);
    battery_update_status(data);
    power_supply_changed(&data->psy);
    power_supply_changed(&data->battery);
    power_supply_changed(&data->usb);
    return IRQ_HANDLED;
}

static int get_usb_property(struct power_supply *usb,enum power_supply_property psp, union power_supply_propval *val)
{
    struct max17048_chip *data = container_of(usb, struct max17048_chip, usb);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = 0;
        if (data->usb_in)
                val->intval = 1;
        data->usb_charger_online = val->intval;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static int get_dc_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
    struct max17048_chip *data = container_of(psy, struct max17048_chip, psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = 0;
        if (data->ta_in)
                val->intval = 1;
        data->charger_online = val->intval;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static int __devinit max17048_probe(struct i2c_client *client, const struct i2c_device_id *id)
{    
    printk("\n enter max17048_probe======");
    int ret,gpio,ta_in,usb_in,retval;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct max17048_chip *chip;
    struct device *dev = &client->dev;
    const struct max17048_platform_data *pdata = dev_get_platdata(dev);

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
        return -EIO;

    chip = kzalloc(sizeof(*chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;


    chip->client = client;
    chip->pdata  = client->dev.platform_data;

    chip->usb_in = 0;
    chip->ta_in  = 0;
    chip->first_delay_count = 0;

    i2c_set_clientdata(client, chip);

    chip->battery.name		= "battery";
    chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
    chip->battery.get_property	= max17048_get_property;
    chip->battery.properties	= max17048_battery_props;
    chip->battery.num_properties= ARRAY_SIZE(max17048_battery_props);

    ret = power_supply_register(&client->dev, &chip->battery);//battery加入power_supply 注册设备文件，这里注册了一个名为battery的 power_supply设备文件，这个设备文件包含了ARRAY_SIZE(max17048_battery_props)个操作
    if (ret) {
        dev_err(&client->dev, "failed: power supply register\n");
        goto battery_failed;
    }

    mutex_init(&chip->work_lock);
    chip->fault = false;
    chip->ta_in = ta_in;
    chip->usb_in = usb_in;
    chip->psy.name = "max8903-dc";
    chip->psy.type = POWER_SUPPLY_TYPE_MAINS; //电源
    chip->psy.get_property = get_dc_property;
    chip->psy.properties   = charger_props;
    chip->psy.num_properties = ARRAY_SIZE(charger_props);
    ret = power_supply_register(dev, &chip->psy);//和battery相似，把ac加入power_supply

    if (ret) {
        dev_err(dev, "failed: power supply register.\n");
        goto err_psy;
    }

    chip->usb.name = "max8903-usb";
    chip->usb.type = POWER_SUPPLY_TYPE_USB; //USB
    chip->usb.get_property = get_usb_property;
    chip->usb.properties = charger_props;
    chip->usb.num_properties = ARRAY_SIZE(charger_props);
    ret = power_supply_register(dev, &chip->usb);//和battery相似，把usb加入power_supply
    if (ret) {
        dev_err(dev, "failed: power supply register.\n");
        goto err_psy;
    }

    max17048_reset(client); //复位
    max17048_get_version(client);

    INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17048_work);//任务宏初始化，max17040加入chip->work队列
    schedule_delayed_work(&chip->work, MAX17048_DELAY);//通过定时器调度队列

    if (pdata->dc_valid == false && pdata->usb_valid == false) {
        dev_err(dev, "No valid power sources.\n");
        printk(KERN_INFO "No valid power sources.\n");
        ret = -EINVAL;
        goto err;
    }
    if (pdata->dc_valid) {
        if (pdata->dok && gpio_is_valid(pdata->dok)) {
            gpio = pdata->dok; /* PULL_UPed Interrupt */
              /* set DOK gpio input */
            ret = gpio_request(gpio, "max8903-DOK");
            if (ret) {
                printk(KERN_ERR"request max8903-DOK error!!\n");
                goto err;
            } else {
                gpio_direction_input(gpio);
            }
                ta_in = gpio_get_value(gpio) ? 0 : 1;
            } else if (pdata->dok && gpio_is_valid(pdata->dok) && pdata->dcm_always_high) {
                ta_in = pdata->dok; /* PULL_UPed Interrupt */
                ta_in = gpio_get_value(gpio) ? 0 : 1;
            } else {
                dev_err(dev, "When DC is wired, DOK and DCM should"
                                " be wired as well."
                                " or set dcm always high\n");
                ret = -EINVAL;
                goto err;
            }
    }
    if (pdata->usb_valid) {
        if (pdata->uok && gpio_is_valid(pdata->uok)) {
            gpio = pdata->uok;
            /* set UOK gpio input */
            ret = gpio_request(gpio, "max8903-UOK");
            if (ret) {
                printk(KERN_ERR"request max8903-UOK error!!\n");
                goto err;
            } else {
               gpio_direction_input(gpio);
            }
            usb_in = gpio_get_value(gpio) ? 0 : 1;
            } else {
               dev_err(dev, "When USB is wired, UOK should be wired."
                            "as well.\n");
               ret = -EINVAL;
               goto err;
            }
    }
    if (pdata->chg) {
        if (!gpio_is_valid(pdata->chg)) {
            dev_err(dev, "Invalid pin: chg.\n");
            ret = -EINVAL;
            goto err;
        }
        /* set CHG gpio input */
        ret = gpio_request(pdata->chg, "max8903-CHG");
        if (ret) {
            printk(KERN_ERR"request max8903-CHG error!!\n");
            goto err;
        } else {
            gpio_direction_input(pdata->chg);
        }
    }
    if (pdata->flt) {
        if (!gpio_is_valid(pdata->flt)) {
            dev_err(dev, "Invalid pin: flt.\n");
            ret = -EINVAL;
            goto err;
        }
        /* set FLT gpio input */
        ret = gpio_request(pdata->flt, "max8903-FLT");
        if (ret) {
            printk(KERN_ERR"request max8903-FLT error!!\n");
            goto err;
        } else {
            gpio_direction_input(pdata->flt);
        }
    }
    if (pdata->usus) {
        if (!gpio_is_valid(pdata->usus)) {
            dev_err(dev, "Invalid pin: usus.\n");
            ret = -EINVAL;
            goto err;
        }
    }

   if (pdata->dc_valid) {
       ret = request_threaded_irq(gpio_to_irq(pdata->dok),
                                    NULL, max8903_dcin,
                                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                                    "MAX8903 DC IN", chip);
       if (ret) {
           dev_err(dev, "Cannot request irq %d for DC (%d)\n",
           gpio_to_irq(pdata->dok), ret);
           goto err_usb_irq;
        }
    }
    if (pdata->usb_valid) {
        ret = request_threaded_irq(gpio_to_irq(pdata->uok),
                                    NULL, max8903_usbin,
                                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                                    "MAX8903 USB IN", chip);
        if (ret) {
            dev_err(dev, "Cannot request irq %d for USB (%d)\n",
            gpio_to_irq(pdata->uok), ret);
            goto err_dc_irq;
        }
    }

    if (pdata->flt) {
        ret = request_threaded_irq(gpio_to_irq(pdata->flt),
                                    NULL, max8903_fault,
                                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                                    "MAX8903 Fault", chip);
    if (ret) {
        dev_err(dev, "Cannot request irq %d for Fault (%d)\n",
        gpio_to_irq(pdata->flt), ret);
        goto err_flt_irq;
        }
    }

    if (pdata->chg) {
        ret = request_threaded_irq(gpio_to_irq(pdata->chg),
                            NULL, max8903_chg,
                            IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                            "MAX8903 Status", chip);
        if (ret) {
            dev_err(dev, "Cannot request irq %d for Status (%d)\n",
            gpio_to_irq(pdata->flt), ret);
            goto err_chg_irq;
        }
    }

    charger_update_status(chip);
    battery_update_status(chip);

    return 0;
err_psy:
    power_supply_unregister(&chip->psy);
battery_failed:
    power_supply_unregister(&chip->battery);
err_usb_irq:
    if (pdata->usb_valid)
        free_irq(gpio_to_irq(pdata->uok), chip);
    cancel_delayed_work(&chip->work);
err_dc_irq:
    if (pdata->dc_valid)
        free_irq(gpio_to_irq(pdata->dok), chip);
    cancel_delayed_work(&chip->work);
err_flt_irq:
    if (pdata->usb_valid)
        free_irq(gpio_to_irq(pdata->uok), chip);
    cancel_delayed_work(&chip->work);
err_chg_irq:
    if (pdata->dc_valid)
        free_irq(gpio_to_irq(pdata->dok), chip);
    cancel_delayed_work(&chip->work);
err:
    if (pdata->uok)
        gpio_free(pdata->uok);
    if (pdata->dok)
        gpio_free(pdata->dok);
    if (pdata->flt)
        gpio_free(pdata->flt);
    if (pdata->chg)
        gpio_free(pdata->chg);
    kfree(chip);
    return ret;
}

static int __devexit max17048_remove(struct i2c_client *client)
{
    struct max17048_chip *chip = i2c_get_clientdata(client);
    power_supply_unregister(&chip->battery);
    cancel_delayed_work(&chip->work);
    kfree(chip);
    return 0;
}


#ifdef CONFIG_PM
static int max17048_suspend(struct i2c_client *client,
                pm_message_t state)
{
    struct max17048_chip *chip = i2c_get_clientdata(client);

    cancel_delayed_work(&chip->work);
    return 0;
}

static int max17048_resume(struct i2c_client *client)
{
    struct max17048_chip *chip = i2c_get_clientdata(client);

    schedule_delayed_work(&chip->work, MAX17048_DELAY);
    return 0;
}

#else
#define max17048_suspend NULL
#define max17048_resume NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id max17048_id[] = {
    { "max17048", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, max17048_id);

static struct i2c_driver max17048_i2c_driver = {
    .driver	= {
            .name	= "max17048",
    },
    .probe		= max17048_probe,
    .remove		= __devexit_p(max17048_remove),
    .suspend	= max17048_suspend,
    .resume		= max17048_resume,
    .id_table	= max17048_id,
};

static int __init max17048_init(void)
{
    return i2c_add_driver(&max17048_i2c_driver);
}

module_init(max17048_init);

static void __exit max17048_exit(void)
{
    i2c_del_driver(&max17048_i2c_driver);
}
module_exit(max17048_exit);

MODULE_AUTHOR("gxl");
MODULE_DESCRIPTION("MAX17048 Fuel Gauge");
MODULE_LICENSE("GPL");








