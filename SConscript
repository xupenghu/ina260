from building import * 
Import('rtconfig')
# get current dir path
cwd = GetCurrentDir()

src = []

# add ina260 src files
src += Glob('ina260.c')
if GetDepend(['PKG_INA260_USING_SENSOR_V1']):
    src += ['ti_ina260_sensor_v1.c']
if GetDepend(['PKG_USING_INA260_EXAMPLE']):
    src += ['example_ina260.c']

# add ina260 inc files
path  = [cwd]
# add src and inc to group 
group = DefineGroup('ina260', src, depend = ['PKG_USING_INA260'], CPPPATH = path)


Return('group')
