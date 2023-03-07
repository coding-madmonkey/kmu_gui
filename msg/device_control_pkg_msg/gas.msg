## GAS COMMAND ##
uint8 NODE_ID # NODE ID : 1, 2, 3, 4
uint8 CMD
# 00 : Nothing
# 01 : GAS SENSOR OFF
# 02 : GAS SENSOR ON

## GAS STATE ##
uint8 STATE
# 00 : Nothing
# 01 : GAS SENSOR OFF
# 02 : GAS SENSOR ON
uint64 DATE # YYYYMMDD
uint64 TIME # HHMMSS
uint8 DATA
# 00 : No data
# 01 : C3H5
# 02 : H2S
