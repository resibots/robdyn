#! /usr/bin/env python
VERSION='0.0.1'
APPNAME='robdyn'

srcdir = '.'
blddir = 'bld'

import copy
import TaskGen
import Options
import os, sys

def init():
    pass

def set_options(opt):
    opt.tool_options('compiler_cxx')
    opt.tool_options('boost')
    opt.tool_options('ode')
    opt.tool_options('eigen')

    opt.add_option('--disable_osg', type='string', help='disable open scene graph support (no visualization)', dest='disable_osg')
    opt.add_option('--64bits', type='string', help='enable 64 bits support', dest='bits64')
    
def configure(conf):
    # log configure options
    fname = blddir + '/configure.options'
    args = open(fname, 'a')
    for i in sys.argv:
        args.write(i + ' ')
    args.write("\n")
    args.close()

    conf.check_tool('compiler_cxx')
    ode_found = conf.check_tool('ode')
    eigen_found = conf.check_tool('eigen')
    conf.check_tool('boost')
    conf.check_boost(lib='', min_version='1.35')
    conf.env['LIB_OSG'] = [ 'osg', 'osgDB', 'osgUtil', 'osgGA', #GLU GL
                           'osgViewer', 'OpenThreads', 
                            'osgFX', 'osgShadow', 'osgTerrain']
    conf.env['LIB_ODE'] += ['pthread']

    if Options.options.disable_osg == "yes":
	osg_flag = ' -DNO_OSG'
	conf.env['NO_OSG'] = True
    else:
	osg_flag = ''
	conf.env['NO_OSG'] = False

    if Options.options.bits64:
	osg_flag += ' -m64  -DBOOST_NO_INTRINSIC_INT64_T '
	conf.env.append_value("LINKFLAGS", "-m64")
    
    common_flags = "-D_REENTRANT -Wall -fPIC -ftemplate-depth-128 -Wno-sign-compare -Wno-deprecated  -Wno-unused " + osg_flag
    cxxflags = conf.env['CXXFLAGS']

    # release
    conf.setenv('default')
    opt_flags = common_flags +  ' -DNDEBUG -O3 -fomit-frame-pointer -finline-functions -ftracer -funroll-loops -fstrict-aliasing -ffast-math'

    opt_flags += ' -mfpmath=sse -march=core2 -msse2'

    conf.env['CXXFLAGS'] = cxxflags + opt_flags.split(' ')

    # debug
    env = conf.env.copy()
    env.set_variant('debug')
    conf.set_env_name('debug', env)
    conf.setenv('debug')
    debug_flags = common_flags + '-O0 -ggdb3 -DDBG_ENABLED'
    conf.env['CXXFLAGS'] = cxxflags + debug_flags.split(' ')



def build(bld):
    print ("Entering into directory " + os.getcwd())
    bld.add_subdirs('src')
    for obj in copy.copy(bld.all_task_gen):
        obj.clone('debug')

def shutdown ():
    pass
