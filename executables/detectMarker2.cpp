/* -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: t;  -*- */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Heni Ben Amor
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <pthread.h>

#include <kinectAR.h>

#include <getopt.h>


#include <iostream>
#include <sns.h>

#include "cparams.h"

using namespace cv;
using namespace std;

CamMode mode;
bool useKinect   = false;
bool useGraphics = false;

const char *opt_chan_tf = "markers";
const char *opt_chan_cam = "video";
char *param_file        = NULL;
char *inp_type          = "normal";

const char *opt_calib = NULL;

/**
 * Main loop
 */
int main(int argc, char **argv) {

    sns_init();
    
    // initialize parameters
    CParams p;
    
    // load from file
    if(param_file != NULL) {
	p = CParams(param_file);
    }
    
    SNS_REQUIRE( opt_calib, "Must specify calibration file\n");
    
    // create a camera processing module
    KinectAR camera(opt_calib, p, opt_chan_cam, opt_chan_tf);
    
    // cancel handlers
    {
	ach_channel_t *chans[] = {&camera.rec.chan, NULL};
	sns_sigcancel( chans, sns_sig_term_default );
    }
    
    
	sns_start();

	// draw image
	while(!sns_cx.shutdown) {

	    // Grab messages from video chan
	    
	    // process video image and draw scene
	    camera.UpdateScene(false);
	    
	    if( sns_cx.shutdown ) break;
	    
	    // detect the marker in the scene
	    camera.DetectMarkers(true);
	    

	    // send the data
	    camera.SendMsg(32);
	    
	    
	    // release memory
	    aa_mem_region_local_release();
	}
	
	SNS_LOG(LOG_NOTICE, "Gracefully halting\n");

	return 0;
}
