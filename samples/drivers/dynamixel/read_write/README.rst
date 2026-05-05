.. zephyr:code-sample:: dynamixel-read-write
   :name: Dynamixel read/write
   :relevant-api: dynamixel

   Ping a Dynamixel servo, read identifying registers, and blink its onboard LED.

Overview
********

This sample exercises the typed Dynamixel API:

#. Pings the servo at id ``1``.
#. Reads ``MODEL_NUMBER``, ``FIRMWARE_VERSION``, ``PRESENT_POSITION``, and
   ``PRESENT_TEMPERATURE`` and logs the results.
#. Blinks the servo's onboard LED three times by writing the ``LED`` register.

The default supported board is :ref:`arduino_mkrzero`. A half-duplex
Dynamixel transceiver is wired to SERCOM5 with the transmit-enable pin on A6
(active-low, matching common DIY transceiver shields).

Wiring
******

::

   SERCOM5 TX  --[transceiver TX]-- DXL D+
   SERCOM5 RX  --[transceiver RX]-- DXL D-
   A6          --[transceiver DIR]-- (low: receive, high: transmit)
   GND         --                    DXL GND
   VIN/VBUS    --                    DXL Vcc (per servo specs)

The default baud is 57600, which matches the factory setting of X-series servos
(XL330, XC330). If you have changed the servo's baud rate, edit ``main.c``
accordingly.

Building and running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/dynamixel/read_write
   :board: arduino_mkrzero
   :goals: build flash
   :compact:

Sample output
*************

.. code-block:: console

   *** Booting Zephyr OS ...
   [00:00:00.012,000] <inf> sample_dynamixel: Pinging servo id=1
   [00:00:00.034,000] <inf> sample_dynamixel: Ping OK
   [00:00:00.045,000] <inf> sample_dynamixel: Model number:     1200
   [00:00:00.056,000] <inf> sample_dynamixel: Firmware version: 47
   [00:00:00.067,000] <inf> sample_dynamixel: Present position: 2048
   [00:00:00.078,000] <inf> sample_dynamixel: Temperature:      29 C
   [00:00:00.089,000] <inf> sample_dynamixel: Blinking LED 3 times
   [00:00:01.890,000] <inf> sample_dynamixel: Sample done
