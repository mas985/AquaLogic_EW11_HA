For P4 users, comment out lines 435 & 437

Add this folder to custom_components/aqualogic_ew11

These lines should be added and customized into configuration.yaml

aqualogic_ew11:
  host: IP ADDRESS HERE
  port: 8899
sensor:
  - platform: aqualogic_ew11
    monitored_conditions:
      - pool_temp
      - air_temp
      - spa_temp
      - pool_chlorinator
      - spa_chlorinator
      - salt_level
      - pump_speed
      - status
switch:
  - platform: aqualogic_ew11
    monitored_conditions:
      - filter
      - lights
      - aux_1
      - aux_2
      - aux_3
      - aux_4
      - aux_5
      - aux_6
      - valve_3
      - filter_low_speed
