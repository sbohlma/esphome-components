copied from: https://github.com/mrtoy-me/esphome-components-test/tree/main/components/vl53l1x

## ESPHome component for VL53L1X and VL53L4CD
This ESPHome external component is based on the Polulo VL53L1X Arduino Library
which in turn was based on the VL53L1X API by STMicroelectronics.
Therefore the licence terms are the same as those presented in the
Polulo VL53L1X Arduino Library.

Polulo VL53L1X Arduino Library states that most of the functionality of their
library is based on the VL53L1X API provided provided by ST (STSW-IMG007),
and some of the explanatory comments are quoted or paraphrased
from the API source code, API user manual (UM2356), and VL53L1X datasheet.
Therefore, the license terms for the API source code (BSD 3-clause
"New" or "Revised" License) also apply to this derivative work, as specified below.

Copyright (c) 2017, STMicroelectronics
Copyright (c) 2018-2022, Pololu Corporation
All Rights Reserved

**All Copyright licences are shown in vl53l1x.cpp and licence.md files**


## Usage: VL53L1X component
This component requires Esphome version 2023.7.1 or later.

The following yaml can be used so ESPHome accesses the component files:
```
external_components:
  - source: github://mrtoy-me/esphome-components-test@main
    components: [ vl53l1x ]
    refresh: 0s
```
This component supports VL53L1X (up to 4000mm range) and VL53L4CD (up to 1300mm range) with default i2c address of 0x29.<BR>
Timing budget (measurement period) is set internally at 500ms. Ranging occurs continuously every 500ms, but measurements are published at the specified update interval. **Note: update interval should be greater or equal to 1 second.**<BR>

The ***vl53l1x:*** configuration allows defining:<BR>
***distance_mode:*** which can be either ***short*** or ***long*** with default ***long***<BR>
***update_interval:*** which defaults to 60s<BR>
**Note: the VL53L4CD sensor can only have distance_mode: short, if VL53L4CD is detected then distance mode is forced to ***short***.**<BR>

Two sensors can be configured ***distance:*** and ***range_status:***<BR>
Distance has units mm while range status gives the status code of the distance measurement.<BR>
**Note: A distance value is returned irrespective of the range status value. It is recommended that a template sensor is used to return the desired value when range status is not valid. See Example YAML below**<BR>

**Note: The range status values defined in this component differ from those used by the Polulo Arduino Library**<BR>
Range status values are as follows:<BR>

0 = Range Valid<BR>

1 = Range valid, no wraparound check fail<BR>

2 = Range valid, below minimum range threshold<BR>

3 = Hardware or VCSEL fail<BR>

4 = Signal fail<BR>
(signal below threshold, too weak for valid measurement)<BR>

5 = Out of bounds fail<BR>
(nothing detected in range typically when target is at or more than maximum range)<BR>

6 = Sigma fail<BR>
(standard deviation is above threshold indicating poor measurement repeatability)<BR>

7 = Wrap target fail<BR>
(target is very reflective and is more than than maximum range)<BR>

8 = Minimum range fail<BR>
(target is below minimum detection threshold)<BR>

9 = Undefined<BR>


## Example YAML
```
external_components:
  - source: github://mrtoy-me/esphome-components-test@main
    components: [ vl53l1x ]
    refresh: 0s

i2c:
    sda: 21
    scl: 22
    scan: true

sensor:
  - platform: vl53l1x
    distance_mode: long
    distance:
      name: My Raw Distance
      id: my_raw_distance
    range_status:
      name: My Range Status
      id: my_range_status
    update_interval: 1s

  - platform: template
    name: My Distance
    device_class: "distance"
    state_class: "measurement"
    unit_of_measurement: "mm"
    accuracy_decimals: 0
    lambda: |-
       if (id(my_range_status).state > 0) {
         return NAN;
       } else {
         return id(my_raw_distance).state;
       }
    update_interval: 1s
```