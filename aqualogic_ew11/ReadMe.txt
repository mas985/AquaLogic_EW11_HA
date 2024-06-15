P4 users, change the following line in core.py to False

    PS8MODE = True # False for P4

Add this folder to custom_components/aqualogic_ew11

These lines should be added and customized into configuration.yaml

logger:
#  default: debug
  logs:
    custom_components.aqualogic_ew11: debug
aqualogic_ew11:
  host: 192.168.0.15
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
      - valve_4      
      - heater_1
      - filter_low_speed
