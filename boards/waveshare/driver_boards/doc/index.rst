.. _ros_driver:
.. _general_driver:

Waveshare ESP32 driver boards
#############################

Overview
********

Two ESP32-based robot controller boards produced by `Waveshare <https://www.waveshare.com/>`_,
sharing one board directory because they share most of a design:

``ros_driver``
   ROS Driver for Robots, as shipped with the RaspRover. ESP32-WROOM-32UE-N4,
   4 MB of flash, an ICM-20948 9-axis IMU at 0x68, and nothing on SPI2.

``general_driver``
   General Driver for Robots. ESP32-WROOM-32UE-N16, 16 MB of flash, a QMI8658C
   6-axis IMU at 0x6b paired with a separate AK09918C magnetometer at 0x0c, an
   unfitted BMP280 footprint at 0x77, and a microSD slot on SPI2.

Everything else is common and lives in ``waveshare_driver_common.dtsi``: the
TB6612 dual H-bridge on LEDC channels 2 and 3 with PCNT encoders, the ST3215
serial bus servo connector on ``uart1`` at 1 Mbps, the SSD1306 128x32 display
at 0x3c and INA219 current monitor at 0x42 on ``i2c0``, two LEDs and the BOOT
button.

.. note::

   The IMUs are not interchangeable in an application: different driver,
   different register map, and the ICM-20948 keeps its AK09916 magnetometer
   behind its own I2C master rather than on the host bus, which is why an
   ``i2c scan`` on ``ros_driver`` shows no 0x0c.

Functionality Overview
**********************

The block diagram below shows the main components and their interconnections.

.. image:: img/esp-wroom-32-pinout.jpg
     :align: center
     :alt: ESP-WROOM32-PINOUT

External I2C connectors
***********************

``ros_driver`` has two 4-pin PH2.0 I2C sockets, P3 and P4 on the schematic,
wired in parallel to the same bus. That bus is ``i2c0``: GPIO32 is SDA and
GPIO33 is SCL. It is shared with the onboard SSD1306 display (0x3c), the INA219
current monitor (0x42) and the ICM-20948 IMU (0x68, behind an LSF0204 level
shifter), so an external device must not use those addresses.

``grove_iic`` is a GPIO nexus over the two pins, for a driver that needs to
bit-bang or recover the bus:

.. code-block:: devicetree

   /* index 0 is SCL (GPIO33), index 1 is SDA (GPIO32) */
   scl-gpios = <&grove_iic 0 GPIO_OPEN_DRAIN>;
   sda-gpios = <&grove_iic 1 GPIO_OPEN_DRAIN>;

.. warning::

   The socket pin order is 1 VDD3V3, 2 GND, 3 SDA, 4 SCL, which is **not** the
   Grove order (1 SCL, 2 SDA, 3 VCC, 4 GND). A Grove cable plugged straight in
   puts 3V3 on the module's SCL pin. The ``grove-header`` binding is used for
   its nexus semantics only, not as a claim of mechanical compatibility.

``general_driver`` does not enable a nexus over its connectors. Its ``i2c0`` is
the same bus on the same pins, shared with the SSD1306 (0x3c), the INA219
(0x42), the QMI8658C IMU (0x6b), the AK09918C magnetometer (0x0c) and the
BMP280 footprint (0x77), so an external device must avoid all five.

microSD, ``general_driver`` only
********************************

The microSD slot is an SD card in SPI mode on ``spi2``: SCLK on GPIO14,
MOSI (CMD) on GPIO13, MISO (D0) on GPIO12. The card's CD/DAT3 pin is the chip
select on GPIO15, driven as a **GPIO** through ``&spi2 cs-gpios`` rather than
routed through pinctrl, because SPI-mode SD holds CS asserted across a
multi-byte command sequence and the hardware CS does not.

``ros_driver`` has no card slot. Its ``spi2`` is enabled with no devices on it
and its chip select *is* routed through pinctrl, so a SPI peripheral added
there that needs CS held across a transaction has to move GPIO15 into
``cs-gpios`` the way ``general_driver`` does.

Flash layout
************

Both boards use Zephyr's shared Espressif AMP partition tables, so the PROCPU
and APPCPU describe the same physical flash and each core's image has a slot
of its own.

``ros_driver`` — ``espressif/partitions_0x1000_amp_4M.dtsi``

===================  ==========  ==========
Partition            Offset      Size
===================  ==========  ==========
``mcuboot``          0x001000    60 KiB
``sys``              0x010000    64 KiB
``image-0``          0x020000    1344 KiB
``image-1``          0x170000    1344 KiB
``image-0-appcpu``   0x2c0000    448 KiB
``image-1-appcpu``   0x330000    448 KiB
``storage``          0x3b0000    192 KiB
``coredump``         0x3ff000    4 KiB
===================  ==========  ==========

``general_driver`` — ``espressif/partitions_0x1000_amp_16M.dtsi``

===================  ==========  ==========
Partition            Offset      Size
===================  ==========  ==========
``mcuboot``          0x001000    60 KiB
``sys``              0x010000    64 KiB
``image-0``          0x020000    5952 KiB
``image-1``          0x5f0000    5952 KiB
``image-0-appcpu``   0xbc0000    1984 KiB
``image-1-appcpu``   0xdb0000    1984 KiB
``storage``          0xfb0000    192 KiB
``coredump``         0xfff000    4 KiB
===================  ==========  ==========

Both tables also carry a pair of 32 KiB ``lpcore`` slots. The ESP32 has no
low-power core, so they are dead weight that comes with the shared table.

Neither table has an ``image-scratch``. MCUboot must therefore run
``CONFIG_BOOT_SWAP_USING_MOVE`` and not ``CONFIG_BOOT_SWAP_USING_SCRATCH``.

.. warning::

   ``general_driver`` adopted this table in place of a hand-written one where
   the application sat at 0x10000 with a 4 MB slot0 and ``storage`` at
   0x810000. **A board programmed with the older layout needs a full erase**,
   not a flash — otherwise MCUboot looks for an image where there is none and
   stale key-value data sits at an address nothing reads:

   .. code-block:: console

      esptool erase-flash

   Anything kept in ``storage`` — WiFi credentials, hostname, application
   settings — is lost and has to be re-provisioned.

System requirements
*******************

Prerequisites
=============

Espressif HAL requires WiFi and Bluetooth binary blobs in order work. Run the command
below to retrieve those files.

.. code-block:: console

   west blobs fetch hal_espressif

.. note::

   It is recommended running the command above after :file:`west update`.

Building & Flashing
*******************

Every example below uses ``ros_driver``. Substitute ``general_driver`` for the
other board; the two take identical commands. Four board targets exist and all
four build:

* ``ros_driver/esp32/procpu``
* ``ros_driver/esp32/appcpu``
* ``general_driver/esp32/procpu``
* ``general_driver/esp32/appcpu``

Simple boot
===========

The board could be loaded using the single binary image, without 2nd stage bootloader.
It is the default option when building the application without additional configuration.

.. note::

   Simple boot does not provide any security features nor OTA updates.

MCUboot bootloader
==================

User may choose to use MCUboot bootloader instead. In that case the bootloader
must be build (and flash) at least once.

There are two options to be used when building an application:

1. Sysbuild
2. Manual build

.. note::

   User can select the MCUboot bootloader by adding the following line
   to the board default configuration file:

   .. code-block:: cfg

      CONFIG_BOOTLOADER_MCUBOOT=y

   ``Kconfig.sysbuild`` in this directory already defaults both boards to
   MCUboot with ``BOOT_SIGNATURE_TYPE_NONE`` under sysbuild.

Sysbuild
========

The sysbuild makes possible to build and flash all necessary images needed to
bootstrap the board with the ESP32 SoC.

To build the sample application using sysbuild use the command:

.. zephyr-app-commands::
   :tool: west
   :app: samples/hello_world
   :board: ros_driver
   :goals: build
   :west-args: --sysbuild
   :compact:

By default, the ESP32 sysbuild creates bootloader (MCUboot) and application
images. But it can be configured to create other kind of images.

Build directory structure created by sysbuild is different from traditional
Zephyr build. Output is structured by the domain subdirectories:

.. code-block::

  build/
  ├── hello_world
  │   └── zephyr
  │       ├── zephyr.elf
  │       └── zephyr.bin
  ├── mcuboot
  │    └── zephyr
  │       ├── zephyr.elf
  │       └── zephyr.bin
  └── domains.yaml

.. note::

   With ``--sysbuild`` option the bootloader will be re-build and re-flash
   every time the pristine build is used.

For more information about the system build please read the :ref:`sysbuild` documentation.

Manual build
============

During the development cycle, it is intended to build & flash as quickly possible.
For that reason, images can be build one at a time using traditional build.

The instructions following are relevant for both manual build and sysbuild.
The only difference is the structure of the build directory.

.. note::

   Remember that bootloader (MCUboot) needs to be flash at least once.

Build and flash applications as usual (see :ref:`build_an_application` and
:ref:`application_run` for more details).

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: ros_driver/esp32/procpu
   :goals: build

The usual ``flash`` target will work with the ``ros_driver`` board
configuration. Here is an example for the :ref:`hello_world`
application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: ros_driver/esp32/procpu
   :goals: flash

Open the serial monitor using the following command:

.. code-block:: shell

   west espressif monitor

After the board has automatically reset and booted, you should see the following
message in the monitor:

.. code-block:: console

   ***** Booting Zephyr OS vx.x.x-xxx-gxxxxxxxxxxxx *****
   Hello World! ros_driver

Debugging
*********

ESP32 support on OpenOCD is available upstream as of version 0.12.0.
Download and install OpenOCD from `OpenOCD`_.

On both boards the JTAG pins are not run to a standard connector (e.g. ARM
20-pin) and need to be manually connected to the external programmer (e.g. a
Flyswatter2):

+------------+-----------+
| ESP32 pin  | JTAG pin  |
+============+===========+
| 3V3        | VTRef     |
+------------+-----------+
| EN         | nTRST     |
+------------+-----------+
| IO14       | TMS       |
+------------+-----------+
| IO12       | TDI       |
+------------+-----------+
| GND        | GND       |
+------------+-----------+
| IO13       | TCK       |
+------------+-----------+
| IO15       | TDO       |
+------------+-----------+

.. warning::

   **JTAG and SPI2 are the same four pins.** IO12, IO13, IO14 and IO15 are
   MISO, MOSI, SCLK and CS on ``spi2``, which on ``general_driver`` is the
   microSD slot. A debug probe and the card cannot both have them: attach the
   probe and SD transfers fail, leave the card in and JTAG is unreliable.
   Disable ``&spi2`` (and on ``general_driver`` the ``sdcard`` node) in an
   overlay for the duration of a JTAG session, or debug over the serial
   console instead.

   This is why ``spim2_default`` lives in each board's own ``-pinctrl.dtsi``
   rather than the shared one — the two boards make different claims on
   GPIO15.

Further documentation can be obtained from the SoC vendor in `JTAG debugging
for ESP32`_.

Here is an example for building the :ref:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: ros_driver/esp32/procpu
   :goals: build flash
   :gen-args: -DOPENOCD=<path/to/bin/openocd> -DOPENOCD_DEFAULT_PATH=<path/to/openocd/share/openocd/scripts>

You can debug an application in the usual way. Here is an example for the :ref:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: ros_driver/esp32/procpu
   :goals: debug

Note on Debugging with GDB Stub
===============================

GDB stub is enabled on ESP32.

* When adding breakpoints, please use hardware breakpoints with command
  ``hbreak``. Command ``break`` uses software breakpoints which requires
  modifying memory content to insert break/trap instructions.
  This does not work as the code is on flash which cannot be randomly
  accessed for modification.

.. _`OpenOCD`: https://github.com/openocd-org/openocd
.. _`JTAG debugging for ESP32`: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/jtag-debugging/index.html

Related Documents
*****************

* `ROS Driver for Robots schematic`_ (PDF)
* `General Driver for Robots wiki`_
* `ESP32 Datasheet`_ (PDF)
* `ESP32-WROOM-32UE Datasheet`_ (PDF) — the N4 is on ``ros_driver``, the N16 on
  ``general_driver``; the modules differ only in flash size
* `ESP32 Hardware Reference`_

.. _ROS Driver for Robots schematic: https://files.waveshare.com/wiki/RaspRover/ROS_Driver_for_Robots.pdf
.. _General Driver for Robots wiki: https://www.waveshare.com/wiki/General_Driver_for_Robots
.. _ESP32 Datasheet: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
.. _ESP32-WROOM-32UE Datasheet: https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf
.. _ESP32 Hardware Reference: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/index.html