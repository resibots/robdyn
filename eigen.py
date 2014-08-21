#! /usr/bin/env python
# encoding: utf-8
# JB Mouret - 2009

"""
Quick n dirty eigen detection
"""

import os, glob, types
import Options, Configure

def detect_eigen(conf):
	env = conf.env
	opt = Options.options

	conf.env['LIB_EIGEN'] = ''
	conf.env['EIGEN_FOUND'] = False
	if Options.options.no_eigen:
		return 0
	if Options.options.eigen:
		conf.env['CPPPATH_EIGEN'] = Options.options.eigen
		conf.env['LIBPATH_EIGEN'] = Options.options.eigen
	else:
		conf.env['CPPPATH_EIGEN'] = ['/usr/include/eigen3', '/usr/local/include/eigen3', '/usr/include', '/usr/local/include']

	res = Configure.find_file('Eigen/Core', conf.env['CPPPATH_EIGEN'])
	conf.check_message('header','Eigen/Core', (res != '') , res)
	if (res == '') :
		return 0
	conf.env['EIGEN_FOUND'] = True
	return 1

def detect(conf):
	return detect_eigen(conf)

def set_options(opt):
	opt.add_option('--eigen', type='string', help='path to eigen', dest='eigen')
	opt.add_option('--no-eigen', type='string', help='disable eigen', dest='no_eigen')
