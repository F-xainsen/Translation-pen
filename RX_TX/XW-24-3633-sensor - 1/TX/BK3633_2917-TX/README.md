<div align='center'>
<font size=10>
BK3633软件发布说明
</font>
</div>

---

## DesignKit发布等级说明:


> A 类：重要问题修复，客户必须修复（修复点做重点说明)。  
> B 类：普通问题修复，客户选择修复。  
> C 类：性能优化，客户可选择修复。  
> D 类：功能增加，客户可选择修复。  
 
| 序号	| 软件版本名称	| 修改内容	| 发布时间	| 发布等级 |
| :---:| :--- | :--- | :--- | :---: 
| 8 | BK3633_DesignKit_V06_2917 | 1、增加app_gatt_codesize_cut工程，删减部分特性，减小使用空间。<br>2、优化RTOS定时逻辑。<br>3、修改鼠标工程WRITE属性相关。|2025/01/21 |	B&D |
| 7 | BK3633_DesignKit_V06_2816 | 1、修复app_mouse 工程2.4G 模式切换到蓝牙可能异常问题。<br>2、更新usb 升级可能出现的异常问题。<br>3、修改主从一体工程连接可能异常问题。<br>4、更新SPI 读数据可能异常问题。<br>5、增加app_gatt_rtos 工程，带freeRTOS。<br>6、adc 低压检测不操作flash 的电压统一设置为2.0V。|2025/01/06 |	B&D |
| 6 | BK3633_DesignKit_V06_2715 | 1、更新 usb 升级失败问题。<br>2、修改鼠标切换地址链接异常问题。 <br>3、修改 mouse 工程 2.4G 和 BLE 切换流程。 <br>4、增加实现主机应用层直接 read rsp、接收从机 notify 数据，使用 APP_DIR_READ_NOTIFY 宏定义开关。。|2024/10/18 |	B&D |
| 5 | BK3633_DesignKit_V06_2614 | 1、默认打开 CHECK_LOW_VOLT_ENABLE宏定义低压时不进行flash擦写。<br>2、修复做主时扫描到异常广播包解析数据可能卡主问题 。<br>3、默认所有工程都关闭随机地址 。|2024/8/28 |	B&C |
| 4 | BK3633_DesignKit_V06_2613 | 1、修改deepsleep唤醒后ble中断可能异常问题。<br>2、修复app_scan应用搜到异常广播包出现解析异常问题 。<br>3、优化RC校准值获取 。<br>4、更新usb 单mic的描述符枚举 。<br>5、修复做主应用可能出现重复绑定问题 。<br>6、修复做主机write操作可能异常问题。<br>7、修复itest功能异常问题。<br>8、增加频偏校准指令。<br>9、增加语音解码功能到app_central工程。<br>10、增加鼠标工程2.4G和BLE切换不重启的功能。|2024/8/10 |	C&D |
| 3 | BK3633_DesignKit_V06_2512 | 1、修改 flash 驱动兼容 GT25Q40 FLASH。<br>2、整合 DUT 测试，灵敏度偏低问题，命令返回异常问题 。<br>3、更新 app_mouse 连接某些电脑,更新链接间隔异常问题 。<br>4、修改 i2c 读取数据可能出现异常的问题 。<br>5、修改扩展广播 buff 大小 。<br>6、优化广播和保持连接的电流，使用 OPTIMIZE_POWER_CONSUMPTION宏控制，适配 app_gatt 工程 。|2024/1/08 |	C&D |
| 2	| BK3633_DesignKit_V06_2411 | 1、 解决小部分手机 update channel map 引起功耗高的问题。 <br>2、 优化 FCC 测试。<br>3、 同时修改 BOOT 和 APP 部分 flash 驱动，适配 8Mbit flash,使用8Mb FLASH 必须修改。 <br>4、 添加 2M 主从模式。 |2024/1/08 |	D |
| 1	| BK3633_DesignKit_V06_2310 | 1、 修改 GCC 编译的一些异常问题。  <br>2、 Gatt 工程增加多绑定功能。<br>3、 增加地址解析功能。 <br>4、 修改 DUT 测试可能不过问题 <br>5、修改 heap 空间可能被异常修改的问题。 |2023/5/30 | C&D |

