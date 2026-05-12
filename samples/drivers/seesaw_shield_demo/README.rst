.. zephyr:code-sample:: seesaw_shield_demo
   :name: Adafruit Seesaw shield demo

   Exercise the rosterloh-drivers Seesaw MFD shields end-to-end.

Overview
********

Builds against any of the ``adafruit_neoslider``,
``adafruit_rotary_encoder``, or ``adafruit_neokey_1x4`` shields and
demonstrates the corresponding driver:

* NeoSlider: reads ADC channels 0-3 to exercise the ADC driver wiring and
  logs the values at 5 Hz. (The slider potentiometer itself is on channel
  18 of the SAMD09 firmware; tweak the demo if you want live slider data.)
* Rotary Encoder: logs ``SENSOR_CHAN_ROTATION`` deltas at 5 Hz.
* NeoKey 1x4: polls GPIO pins 4-7 and logs key down/up events.

If the active shield ships NeoPixels, the demo also cycles the strip
through red/green/blue.

Building and Running
********************

.. code-block:: console

   west build -p always \
     -b adafruit_qt_py_esp32s3/esp32s3/procpu \
     --shield adafruit_neoslider \
     samples/drivers/seesaw_shield_demo
   west flash
