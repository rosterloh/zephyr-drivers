.. zephyr:code-sample:: dynamixel-group-io
   :name: Dynamixel group I/O
   :relevant-api: dynamixel

   Drive multiple Dynamixel servos in single transactions using SYNC_READ,
   SYNC_WRITE, BULK_READ and BULK_WRITE.

Overview
********

This sample exercises the four Protocol-2 group instructions exposed by the
typed Dynamixel API:

#. ``dxl_sync_write_u8`` — broadcasts ``TORQUE_ENABLE`` /
   ``OPERATING_MODE`` / ``TORQUE_ENABLE`` to every servo on the bus in
   three frames instead of ``N × 3`` unicasts.
#. ``dxl_bulk_write`` — writes a different register per entry
   (``PROFILE_VELOCITY`` and ``LED``) for every servo in one transaction.
#. ``dxl_sync_write_u32`` — broadcasts ``GOAL_POSITION`` to drive every
   servo to the same target.
#. ``dxl_sync_read_u32`` — pulls ``PRESENT_POSITION`` from every servo
   and surfaces per-slot results via the ``errs[]`` array.
#. ``dxl_bulk_read`` — reads ``PRESENT_POSITION`` (32-bit) and
   ``PRESENT_TEMPERATURE`` (8-bit) per servo in a single mixed-width
   transaction.

The sample sweeps the servos between two targets, reads the positions back,
queries a status snapshot, then parks the bus (torque + LED off) with a
final ``dxl_bulk_write``.

Servos are discovered from devicetree via ``dxl_motor_get()`` /
``dxl_motor_count()``; unreachable ids are filtered out at start-up so the
group transactions only address servos that responded to ``dxl_ping``.

The default supported board is :ref:`arduino_mkrzero`. A half-duplex
Dynamixel transceiver is wired to SERCOM5 with the transmit-enable pin on A6
(active-low, matching common DIY transceiver shields). The overlay declares
two servos (``id = 1`` and ``id = 2``); the sample still runs with a single
servo on the bus — the unreachable id is reported in the per-slot
``errs[]`` output.

Wiring
******

::

   SERCOM5 TX  --[transceiver TX]-- DXL D+
   SERCOM5 RX  --[transceiver RX]-- DXL D-
   A6          --[transceiver DIR]-- (low: receive, high: transmit)
   GND         --                    DXL GND
   VIN/VBUS    --                    DXL Vcc (per servo specs)

The default baud is 57600, which matches the factory setting of X-series
servos (XL330, XC330). If you have changed your servos' baud rate or ids,
edit the overlay and ``main.c`` accordingly.

Building and running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/dynamixel/group_io
   :board: arduino_mkrzero
   :goals: build flash
   :compact:

Sample output
*************

.. code-block:: console

   *** Booting Zephyr OS ...
   [00:00:00.012,000] <inf> sample_dynamixel_group: Discovered 2 servo(s)
   [00:00:00.034,000] <inf> sample_dynamixel_group: == SYNC_WRITE: TORQUE_ENABLE=0, OPERATING_MODE=POSITION, TORQUE_ENABLE=1 across 2 servo(s)
   [00:00:00.067,000] <inf> sample_dynamixel_group: == BULK_WRITE: PROFILE_VELOCITY=60 + LED=1, 4 entries
   [00:00:00.090,000] <inf> sample_dynamixel_group: -- SYNC_WRITE: GOAL_POSITION=1024 to all servos
   [00:00:00.601,000] <inf> sample_dynamixel_group: == SYNC_READ: PRESENT_POSITION across 2 servo(s)
   [00:00:00.612,000] <inf> sample_dynamixel_group:    servo id=1 position=1031
   [00:00:00.623,000] <inf> sample_dynamixel_group:    servo id=2 position=1029
   [00:00:00.640,000] <inf> sample_dynamixel_group: -- SYNC_WRITE: GOAL_POSITION=3072 to all servos
   [00:00:01.151,000] <inf> sample_dynamixel_group: == SYNC_READ: PRESENT_POSITION across 2 servo(s)
   [00:00:01.162,000] <inf> sample_dynamixel_group:    servo id=1 position=3070
   [00:00:01.173,000] <inf> sample_dynamixel_group:    servo id=2 position=3074
   [00:00:01.190,000] <inf> sample_dynamixel_group: == BULK_READ: PRESENT_POSITION + PRESENT_TEMPERATURE, 4 entries
   [00:00:01.201,000] <inf> sample_dynamixel_group:    servo id=1 position=3070
   [00:00:01.212,000] <inf> sample_dynamixel_group:    servo id=1 temperature=29
   [00:00:01.223,000] <inf> sample_dynamixel_group:    servo id=2 position=3074
   [00:00:01.234,000] <inf> sample_dynamixel_group:    servo id=2 temperature=30
   [00:00:01.250,000] <inf> sample_dynamixel_group: == BULK_WRITE: park (TORQUE_ENABLE=0 + LED=0)
   [00:00:01.270,000] <inf> sample_dynamixel_group: Sample done
