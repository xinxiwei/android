#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/power/max17055_battery.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/sort.h>

//参照充电图
#define LOW_VOLT_THRESHOLD	(3400000)  
#define HIGH_VOLT_THRESHOLD	(4200000)
 
#define  DESIGNCAP              (0x2710)     //5000mAh
#define  ICHGTERM               (0x0020)     //5mA
#define  VEMPTY                 (0xAA61)    //3.4V
#define  CHARGEVOLTAGE          (4.2)       //充电电压

#define  STATUES_REG            (0x00) //状态寄存器
#define  RECAP_REG              (0x05) //容量寄存器
#define  RESOC_REG              (0x06) //百分比寄存器
#define  FSTA_REG               (0x3D)
#define MAX17055_DELAY		    (1000)
#define MAX17055_BATTERY_FULL	(95)

struct max17055_chip {
    struct i2c_client	            *client;
    struct max17055_platform_data   *pdata;
    struct delayed_work	            work;
    struct device                   *dev;
    struct power_supply	            battery; //表示添加电池供电
    
    int online;                 //State Of Connect
    int vcell;                 //battery voltage 电池电压
    int soc;                  //当前电量百分比
    int temperature;

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

static void charger_update_status(struct max17055_chip *data);
static void battery_update_status(struct max17055_chip *data);
static int max17055_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
    int ret;
    ret = i2c_smbus_write_byte_data(client, reg, value);
    if (ret < 0)
        dev_err(&client->dev, "%s: err %d\n", __func__, ret);
    return ret;
}

static int max17055_read_reg(struct i2c_client *client, u8 reg)
{
    int ret;
    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0)
        dev_err(&client->dev, "%s: err %d\n", __func__, ret);
    return ret;
}

void WriteAndVerifyRegister (struct i2c_client *client, u8 reg, u16 value)
{
    u16 tmp = 0;
    u16 readvalue = 0;
	
    do{
          max17055_write_reg(client , reg ,value);
          mdelay(1);
          readvalue = max17055_read_reg( client,reg);
          tmp++;
     }while( (value !=  readvalue) && (tmp < 3));
}

void max17055_initial(struct i2c_client *client )
{
    u16  status_value ,temp ; 
    u16  hibcfg ,tmpvalue, status; 
    
	//step1-check for power
	status_value = max17055_read_reg(client, STATUES_REG);
    if( !(status_value & 0x0002))
	{    
        temp  =  max17055_read_reg(client, RECAP_REG);
        temp  =  max17055_read_reg(client, RESOC_REG);
        return ;
	}
    ////step2- wait for the max17055 to complete its startup operation
    while( max17055_read_reg(client, FSTA_REG) & 1 )
    {
        mdelay(10);                    //dalay 10ms            
    }

   //step3- EZ config ( no ini file is need)
    max17055_write_reg(client, 0x18, DESIGNCAP);
    max17055_write_reg(client, 0x45, DESIGNCAP/32);
    max17055_write_reg(client, 0x1E, ICHGTERM);
    max17055_write_reg(client, 0x3A, VEMPTY);
    hibcfg =  max17055_read_reg(client, 0xBA);

    max17055_write_reg(client, 0x60, 0x90);
    max17055_write_reg(client, 0xBA, 0);
    max17055_write_reg(client, 0x60, 0);
    if( CHARGEVOLTAGE > 4.275 )
    {
        tmpvalue = (DESIGNCAP/32)*51200/DESIGNCAP;
        max17055_write_reg(client, 0x46, tmpvalue);
        max17055_write_reg(client, 0xDB, 0x8400);
    }else{
        tmpvalue = (DESIGNCAP/32)*44138/DESIGNCAP;
        max17055_write_reg(client, 0x46, tmpvalue);
        max17055_write_reg(client, 0xDB, 0x8000);
    }
    while(max17055_read_reg(client, 0xDB)&0x8000)
    {
        mdelay(10);
    }
    max17055_write_reg(client, 0xBA ,hibcfg);        //max17055_write_reg(client, 0xDB, hibcfg)
        
   // step4 - initialzation complete
    status = max17055_read_reg(client, 0x00);
    WriteAndVerifyRegister(client, 0x00, status & 0xFFFD);
   
    return ;
}

static int max17055_get_property(struct power_supply *psy,  enum power_supply_property psp,  union power_supply_propval *val)
{
    struct max17055_chip *chip = container_of(psy, struct max17055_chip, battery);

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

static void max17055_work(struct work_struct *work)
{
    struct max17055_chip *chip;

    chip = container_of(work, struct max17055_chip, work.work);

    chip ->soc   = max17055_read_reg(chip->client, 0x06);//电量百分比
    chip ->vcell = max17055_read_reg(chip->client, 0x19); //电池电压
    chip ->temperature = max17055_read_reg(chip->client, 0x08); //温度寄存器

    charger_update_status(chip);
    battery_update_status(chip);

    schedule_delayed_work(&chip->work, MAX17055_DELAY);
}

static enum power_supply_property max17055_battery_props[] = {

    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
    POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static enum power_supply_property charger_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static void charger_update_status(struct max17055_chip *data)
{
    if (data->usb_in || data->ta_in) {
        if (data->ta_in)
            data->charger_online = 1;
        if (data->usb_in)
            data->usb_charger_online = 1;
    } else {
        data->charger_online = 0;
        data->usb_charger_online = 0;
    }
    if (data->charger_online == 0 && data->usb_charger_online == 0) {
        data->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
        if (gpio_get_value(data->pdata->chg) == 0) {
            data->battery_status = POWER_SUPPLY_STATUS_CHARGING;
        } else if (data->ta_in && gpio_get_value(data->pdata->chg) == 1) {
            if (!data->pdata->feature_flag) {
                if (data->soc >= 99)
                    data->battery_status = POWER_SUPPLY_STATUS_FULL;
                else
                    data->battery_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
                } else {
                    data->battery_status = POWER_SUPPLY_STATUS_FULL;
                }
            } else if (data->usb_in && gpio_get_value(data->pdata->chg) == 1) {
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

static void battery_update_status(struct max17055_chip *data)
{
    int temp = 0;
    static int temp_last;
    bool changed_flag;


    changed_flag = false;
    mutex_lock(&data->work_lock);
    if (!data->pdata->feature_flag) {
		data ->vcell = max17055_read_reg(data->client, 0x19);
        if (temp_last == 0) {
            data->vcell = temp;
            temp_last = temp;
        }
        if (data->charger_online == 0 && temp_last != 0) {
            temp_last = temp;
            data->vcell = temp;
        }
        if (data->charger_online == 1 || data->usb_charger_online == 1) {
            data->vcell = temp;
            temp_last = temp;
        }
		data ->soc   = max17055_read_reg(data->client, 0x06);
        if (data->soc != data->old_percent) {
            data->old_percent = data->soc;
            changed_flag = true;
        }
        if (changed_flag) {
            changed_flag = false;
            power_supply_changed(&data->battery);
        }
         /*
          *because boot time gap between led framwork and charger
          *framwork,when system boots with charger attatched,
          *charger led framwork loses the first charger online event,
          *add once extra power_supply_changed can fix this issure
          */
        if (data->first_delay_count < 200) {
            data->first_delay_count = data->first_delay_count + 1 ;
            power_supply_changed(&data->battery);
        }
    }
    mutex_unlock(&data->work_lock);
}

static irqreturn_t max8903_dcin(int irq, void *_data)
{
    struct max17055_chip *data = _data;
    struct max17055_platform_data *pdata = data->pdata;
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
    struct max17055_chip *data = _data;
    struct max17055_platform_data *pdata = data->pdata;
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
    struct max17055_chip *data = _data;
    struct max17055_platform_data *pdata = data->pdata;
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
    struct max17055_chip *data = _data;
    struct max17055_platform_data *pdata = data->pdata;
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

static int get_usb_property(struct power_supply *usb,  enum power_supply_property psp,  union power_supply_propval *val)
{
    struct max17055_chip *data = container_of(usb, struct max17055_chip, usb);

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

static int get_dc_property(struct power_supply *psy,  enum power_supply_property psp,  union power_supply_propval *val)
{
    struct max17055_chip *data = container_of(psy,  struct max17055_chip, psy);

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

static int __devinit max17055_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
    int ret,gpio,ta_in,usb_in,retval;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct max17055_chip *chip;
    struct device *dev = &client->dev;
    const struct max17055_platform_data *pdata = dev_get_platdata(dev);

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
    chip->battery.get_property	= max17055_get_property;
    chip->battery.properties	= max17055_battery_props;
    chip->battery.num_properties= ARRAY_SIZE(max17055_battery_props);

    ret = power_supply_register(&client->dev, &chip->battery);
    if (ret) {
        dev_err(&client->dev, "failed: power supply register\n");
        goto battery_failed;
    }

    mutex_init(&chip->work_lock);
    chip->fault = false;
    chip->ta_in = ta_in;
    chip->usb_in = usb_in;
    chip->psy.name = "max8903-dc";
    chip->psy.type = POWER_SUPPLY_TYPE_MAINS;
    chip->psy.get_property = get_dc_property;
    chip->psy.properties   = charger_props;
    chip->psy.num_properties = ARRAY_SIZE(charger_props);
    ret = power_supply_register(dev, &chip->psy);

    if (ret) {
        dev_err(dev, "failed: power supply register.\n");
        goto err_psy;
    }

    chip->usb.name = "max8903-usb";
    chip->usb.type = POWER_SUPPLY_TYPE_USB;
    chip->usb.get_property = get_usb_property;
    chip->usb.properties = charger_props;
    chip->usb.num_properties = ARRAY_SIZE(charger_props);
    ret = power_supply_register(dev, &chip->usb);
    if (ret) {
        dev_err(dev, "failed: power supply register.\n");
        goto err_psy;
    }

    
    max17055_initial(client );
    INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17055_work);
    schedule_delayed_work(&chip->work, MAX17055_DELAY);

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

static int __devexit max17055_remove(struct i2c_client *client)
{
    struct max17055_chip *chip = i2c_get_clientdata(client);

    power_supply_unregister(&chip->battery);
    cancel_delayed_work(&chip->work);
    kfree(chip);
    return 0;
}


#ifdef CONFIG_PM

static int max17055_suspend(struct i2c_client *client,
                pm_message_t state)
{
    struct max17055_chip *chip = i2c_get_clientdata(client);

    cancel_delayed_work(&chip->work);
    return 0;
}

static int max17055_resume(struct i2c_client *client)
{
    struct max17055_chip *chip = i2c_get_clientdata(client);

    schedule_delayed_work(&chip->work, MAX17055_DELAY);
    return 0;
}

#else

#define max17055_suspend NULL
#define max17055_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17055_id[] = {
    { "max17055", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, max17055_id);

static struct i2c_driver max17055_i2c_driver = {
    .driver	= {
            .name	= "max17055",
    },
    .probe		= max17055_probe,
    .remove		= __devexit_p(max17055_remove),
    .suspend	= max17055_suspend,
    .resume		= max17055_resume,
    .id_table	= max17055_id,
};

static int __init max17055_init(void)
{
    return i2c_add_driver(&max17055_i2c_driver);
}
module_init(max17055_init);

static void __exit max17055_exit(void)
{
    i2c_del_driver(&max17055_i2c_driver);
}
module_exit(max17055_exit);

MODULE_AUTHOR("gxl");
MODULE_DESCRIPTION("MAX17055 Fuel Gauge");
MODULE_LICENSE("GPL");








