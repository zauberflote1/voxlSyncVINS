/*******************************************************************************
 * Copyright 2023 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <list>
#include <algorithm>
#include <modal_json.h>

#include "config_file.h"
#include "config_defaults.h"
#include "common_defs.h"
#include <modal_journal.h>

using namespace std;

#define contains(a, b) (std::find(a.begin(), a.end(), b) != a.end())


#define CONFIG_FILE_HEADER "\
/**\n\
 * voxl-camera-server Configuration File\n\
 *\n\
 * Each camera has configurations for up to 4 HAL3 streams:\n\
 *    - `preview` stream for raw unprocessed images from CV cameras\n\
 *    - `small_video` 720p (ish) h264/h265 compressed for fpv video streaming\n\
 *    - `large_video` 4k (ish) h264/h265 for onboard video recording to disk\n\
 *    - `snapshot` ISP-processed JPG snapshots that get saved to disk\n\
 *\n\
 * on QRB5165 platforms (VOXL2 and VOXL2 mini) you can only have 3 of the 4 enabled\n\
 *\n\
 * This file is generated from default values by voxl-configure-cameras.\n\
 * Do not expect arbitrary resolutions to work, the ISP and video compression\n\
 * pipelines only support very specific resolutions.\n\
 *\n\
 * The default video compression mode is cqp or Constant Quantization Parameter\n\
 *\n\
 *\n\
 *\n\
 */\n"




int config_file_print(PerCameraInfo* cams, int n)
{
	printf("=================================================================\n");
	printf("configuration for %d cameras:\n", n);
	printf("\n");
	for(int i=0; i<n; i++){
		printf("cam #%d\n", i);
		printf("    name:                %s\n", cams[i].name);
		printf("    sensor type:         %s\n", GetTypeString(cams[i].type));
		printf("    isEnabled:           %d\n", cams[i].isEnabled);
		printf("    camId:               %d\n", cams[i].camId);
		printf("    camId2:              %d\n", cams[i].camId2);
		printf("    fps:                 %d\n", cams[i].fps);
		printf("    en_rotate:           %d\n", cams[i].en_rotate);
		printf("    en_rotate2:          %d\n", cams[i].en_rotate2);
		printf("\n");
		printf("    en_preview:          %d\n", cams[i].en_preview);
		printf("    pre_width:           %d\n", cams[i].pre_width);
		printf("    pre_height:          %d\n", cams[i].pre_height);
		printf("    en_raw_preview:      %d\n", cams[i].en_raw_preview);
		printf("\n");
		printf("    en_small_video:      %d\n", cams[i].en_small_video);
		printf("    small_video_width:   %d\n", cams[i].small_video_width);
		printf("    small_video_height:  %d\n", cams[i].small_video_height);
		printf("\n");
		printf("    en_large_video:      %d\n", cams[i].en_large_video);
		printf("    large_video_width:   %d\n", cams[i].large_video_width);
		printf("    large_video_height:  %d\n", cams[i].large_video_height);
		printf("\n");
		printf("    en_snapshot:         %d\n", cams[i].en_snapshot);
		printf("    snap_width:          %d\n", cams[i].snap_width);
		printf("    snap_height:         %d\n", cams[i].snap_height);
		printf("\n");
		printf("    ae_mode:             %s\n", GetAEModeString(cams[i].ae_mode));
		printf("    standby_enabled:     %d\n", cams[i].standby_enabled);
		printf("    decimator:           %d\n", cams[i].decimator);
		printf("    independent_exposure:%d\n", cams[i].ind_exp);
		printf("\n");
	}
	printf("=================================================================\n");
	return 0;
}



// swap height and width if necessary (common mistake when typing in config file)
static void _check_and_swap_width_height(int* w, int* h)
{
	// all is well
	if(*h <= *w) return;

	M_WARN("detected swapped height and width %d %d in config file\n", *h, *w);
	M_WARN("automatically switching them\n");

	int height = *h;
	int width = *w;

	*h = width;
	*w = height;
	return;
}



// -----------------------------------------------------------------------------------------------------------------------------
// Read and parse the config file. This function can be modified to support any config file format. The information for each
// camera read from the config file is returned from this function.
//
// Note:
// "cameras" will contain memory allocated by this function and it is the callers responsibility to free/pop it it.
// -----------------------------------------------------------------------------------------------------------------------------
Status ReadConfigFile(PerCameraInfo* cameras, int* camera_len)
{
	int i, tmp, is_writing_fresh;
	std::list<int> cameraIds;
	std::list<string> cameraNames;

	const char* sensor_strings[] = SENSOR_STRINGS;
	const char* ae_strings[] = AE_STRINGS;
	const char* venc_mode_strings[] = VENC_MODE_STRINGS;
	const char* venc_control_strings[] = VENC_CONTROL_STRINGS;


	cJSON* parent = NULL;
	cJSON* cameras_json = NULL;
	int numCameras = 0;

	// caller provided a list of cameras, must be writing a new file from cam config helper
	if(*camera_len>0){
		is_writing_fresh = 1;
		numCameras = *camera_len;
		remove(CONFIG_FILE_NAME);
		parent = cJSON_CreateObject();
		cJSON_AddNumberToObject(parent, "version", CURRENT_VERSION);
		tmp = 0;
		cameras_json = json_fetch_array_and_add_if_missing(parent, "cameras", &tmp);
	}
	// normal reading mode
	else{
		is_writing_fresh = 0;
		parent = json_read_file(CONFIG_FILE_NAME);
		if(parent==NULL){
			M_ERROR("missing config file\n");
			return S_ERROR;
		}
		cameras_json = json_fetch_array_of_objects_and_add_if_missing(parent, "cameras", &numCameras);
	}

	// sanity check
	if(numCameras<1 || numCameras > 7){
		fprintf(stderr, "array of cameras should be between 1 and 7, found %d\n", numCameras);
		return S_ERROR;
	}

	// now go through all the cameras, preset or empty
	for(i=0; i<numCameras; i++){
		PerCameraInfo* cam = &cameras[i];

		cJSON* item;

		// if writing fresh, we have caminfo structs but need to create json array items
		if(is_writing_fresh){
			item = cJSON_CreateObject();
			cJSON_AddItemToArray(cameras_json, item);
		}
		// if not writing fresh, we fetch the array item and start with a clean camInfo struct
		else{
			item = cJSON_GetArrayItem(cameras_json, i);
			*cam = getDefaultCameraInfo(SENSOR_INVALID);
		}

		if(item==NULL){
			M_ERROR("failed to fetch item %d from json array\n", i);
			goto ERROR_EXIT;
		}

		// if writing fresh, this name will have been set by the config helper
		if(json_fetch_enum_with_default(item, "type", (int*)&cam->type, sensor_strings, SENSOR_MAX_TYPES, (int)cam->type)){
			M_ERROR("failed to parse type for camera %d\n", i);
			goto ERROR_EXIT;
		}
		//printf("FOUND CAM TYPE %s\n", sensor_strings[cam->type]);

		// if not writing fresh, reset the whole cam info struct to default
		if(!is_writing_fresh){
			//printf("getting fresh defaults for cam type %s\n", sensor_strings[cam->type]);
			*cam = getDefaultCameraInfo(cam->type);
		}

		if(json_fetch_string_with_default(item, "name", cam->name, 63, cam->name)){
			M_ERROR("Reading config file: camera name not specified\n", cam->name);
			goto ERROR_EXIT;
		}

		// record the camera name separately to make sure there are no duplicates
		if(contains(cameraNames, cam->name)){
			M_ERROR("Reading config file: multiple cameras with name: %s\n", cam->name);
			goto ERROR_EXIT;
		}
		cameraNames.push_back(cam->name);

		json_fetch_bool_with_default(item, "enabled", &tmp, cam->isEnabled);
		cam->isEnabled = tmp;

		// check cam id1 is present and not a duplicate
		if(json_fetch_int_with_default(item, "camera_id", &(cam->camId), cam->camId)){
			M_ERROR("Reading config file: camera id not specified for: %s\n", cam->name);
			goto ERROR_EXIT;
		}
		if(cam->isEnabled){
			if(contains(cameraIds, cam->camId)){
				M_ERROR("Reading config file: multiple cameras with id: %d\n", cam->camId);
				goto ERROR_EXIT;
			}
			cameraIds.push_back(cam->camId);
		}


		// check cam id2 is present and not a duplicate
		if(cJSON_HasObjectItem(item, "camera_id_second") || cam->camId2>=0){
			json_fetch_int_with_default(item, "camera_id_second", &(cam->camId2), cam->camId2);
			M_DEBUG("stereo camera \"%s\"with cam ids %d & %d\n", cam->name, cam->camId, cam->camId2);
			if(cam->isEnabled && contains(cameraIds, cam->camId2)){
				M_ERROR("Reading config file: multiple cameras with id: %d\n", cam->camId2);
				goto ERROR_EXIT;
			}
		}

		// if in stereo mode, also show the independent exposure flag
		if(cam->camId2>=0){
			M_DEBUG("Secondary id found for camera: %s, assuming stereo\n", cam->name);
			if(cam->isEnabled) cameraIds.push_back(cam->camId2);
			json_fetch_bool_with_default(item, "independent_exposure",  &cam->ind_exp, cam->ind_exp);
		}

		// framerate is simple
		json_fetch_int_with_default(item, "fps" , &cam->fps, cam->fps);

		// rotation only for CV cameras
		if( cam->type == SENSOR_OV7251 || \
			cam->type == SENSOR_OV9782 || \
			cam->type == SENSOR_AR0144)
		{
			json_fetch_bool_with_default (item, "en_rotate",         &cam->en_rotate,  cam->en_rotate);
		}

		if(cam->camId2>=0)
		{
			if ((cam->type == SENSOR_OV7251) ||
				(cam->type == SENSOR_OV9782) ||
				(cam->type == SENSOR_AR0144))
			{
				json_fetch_bool_with_default (item, "en_rotate_second", &cam->en_rotate2, cam->en_rotate);
			}
		}


		// now we parse the 4 streams, preview, small, large video, and snapshot
		// only populate and parse if enabled by default or explicitly set by the user

		// don't know what this was for??
		//cJSON_GetObjectItem(item, "en_preview");


		if(cam->type != SENSOR_TOF){
			json_fetch_bool_with_default (item, "en_preview",         &cam->en_preview,  cam->en_preview);
			json_fetch_int_with_default  (item, "preview_width",      &cam->pre_width,   cam->pre_width);
			json_fetch_int_with_default  (item, "preview_height",     &cam->pre_height,  cam->pre_height);
			json_fetch_bool_with_default (item, "en_raw_preview",     &cam->en_raw_preview, cam->en_raw_preview);
			_check_and_swap_width_height(&cam->pre_width, &cam->pre_height);
		}


		if(cJSON_GetObjectItem(item, "en_small_video")!=NULL || cam->en_small_video){
			json_fetch_bool_with_default(item, "en_small_video",      &cam->en_small_video,      cam->en_small_video);
			json_fetch_int_with_default (item, "small_video_width",   &cam->small_video_width,   cam->small_video_width);
			json_fetch_int_with_default (item, "small_video_height",  &cam->small_video_height,  cam->small_video_height);
			json_fetch_enum_with_default(item, "small_venc_mode",    (int*)&cam->small_venc_config.mode,    venc_mode_strings, VENC_MAXMODES, (int)cam->small_venc_config.mode);
			json_fetch_enum_with_default(item, "small_venc_br_ctrl", (int*)&cam->small_venc_config.br_ctrl, venc_control_strings, VENC_MAXMODES, (int)cam->small_venc_config.br_ctrl);
			json_fetch_int_with_default (item, "small_venc_Qfixed",   &cam->small_venc_config.Qfixed,   cam->small_venc_config.Qfixed);
			json_fetch_int_with_default (item, "small_venc_Qmin",     &cam->small_venc_config.Qmin,     cam->small_venc_config.Qmin);
			json_fetch_int_with_default (item, "small_venc_Qmax",     &cam->small_venc_config.Qmax,     cam->small_venc_config.Qmax);
			json_fetch_int_with_default (item, "small_venc_nPframes", &cam->small_venc_config.nPframes, cam->small_venc_config.nPframes);
			json_fetch_double_with_default(item, "small_venc_mbps",   &cam->small_venc_config.mbps,     cam->small_venc_config.mbps);
			_check_and_swap_width_height(&cam->small_video_width, &cam->small_video_height);
		}

		if(cJSON_GetObjectItem(item, "en_large_video")!=NULL || cam->en_large_video){
			json_fetch_bool_with_default (item, "en_large_video",      &cam->en_large_video,      cam->en_large_video);
			json_fetch_int_with_default  (item, "large_video_width",   &cam->large_video_width,   cam->large_video_width);
			json_fetch_int_with_default  (item, "large_video_height",  &cam->large_video_height,  cam->large_video_height);
			json_fetch_enum_with_default(item, "large_venc_mode",    (int*)&cam->large_venc_config.mode,    venc_mode_strings, VENC_MAXMODES, (int)cam->large_venc_config.mode);
			json_fetch_enum_with_default(item, "large_venc_br_ctrl", (int*)&cam->large_venc_config.br_ctrl, venc_control_strings, VENC_MAXMODES, (int)cam->large_venc_config.br_ctrl);
			json_fetch_int_with_default (item, "large_venc_Qfixed",   &cam->large_venc_config.Qfixed,   cam->large_venc_config.Qfixed);
			json_fetch_int_with_default (item, "large_venc_Qmin",     &cam->large_venc_config.Qmin,     cam->large_venc_config.Qmin);
			json_fetch_int_with_default (item, "large_venc_Qmax",     &cam->large_venc_config.Qmax,     cam->large_venc_config.Qmax);
			json_fetch_int_with_default (item, "large_venc_nPframes", &cam->large_venc_config.nPframes, cam->large_venc_config.nPframes);
			json_fetch_double_with_default(item, "large_venc_mbps",   &cam->large_venc_config.mbps,     cam->large_venc_config.mbps);
			_check_and_swap_width_height(&cam->large_video_width, &cam->large_video_height);
		}

		if(cJSON_GetObjectItem(item, "en_snapshot")!=NULL || cam->en_large_video){
			json_fetch_bool_with_default (item, "en_snapshot",        &cam->en_snapshot, cam->en_snapshot);
			json_fetch_int_with_default  (item, "en_snapshot_width",  &cam->snap_width,  cam->snap_width);
			json_fetch_int_with_default  (item, "en_snapshot_height", &cam->snap_height, cam->snap_height);
			_check_and_swap_width_height(&cam->pre_width, &cam->pre_height);
		}

		if(json_fetch_enum_with_default(item, "ae_mode", (int*)&cam->ae_mode, ae_strings, AE_MAX_MODES, (int)cam->ae_mode)){
			goto ERROR_EXIT;
		}

		// only load histogram settings if enabled (not used by default anymore)
		if(cam->ae_mode == AE_LME_HIST){
			json_fetch_float_with_default (item, "ae_desired_msv", &cam->ae_hist_info.desired_msv, cam->ae_hist_info.desired_msv);
			json_fetch_float_with_default (item, "ae_k_p_ns",      &cam->ae_hist_info.k_p_ns,      cam->ae_hist_info.k_p_ns);
			json_fetch_float_with_default (item, "ae_k_i_ns",      &cam->ae_hist_info.k_i_ns,      cam->ae_hist_info.k_i_ns);
			json_fetch_float_with_default (item, "ae_max_i",       &cam->ae_hist_info.max_i,       cam->ae_hist_info.max_i);
		}

		// only load msv settings if enabled (default for all but hires cams)
		if(cam->ae_mode == AE_LME_MSV){
			json_fetch_float_with_default (item, "ae_desired_msv",       &cam->ae_msv_info.desired_msv,                       cam->ae_msv_info.desired_msv);
			json_fetch_int_with_default   (item, "exposure_min_us",      (int*)&cam->ae_msv_info.exposure_min_us,             (int)cam->ae_msv_info.exposure_min_us);
			json_fetch_int_with_default   (item, "exposure_max_us",      (int*)&cam->ae_msv_info.exposure_max_us,             (int)cam->ae_msv_info.exposure_max_us);
			json_fetch_int_with_default   (item, "gain_min",             (int*)&cam->ae_msv_info.gain_min,                    (int)cam->ae_msv_info.gain_min);
			json_fetch_int_with_default   (item, "gain_max",             (int*)&cam->ae_msv_info.gain_max,                    (int)cam->ae_msv_info.gain_max);
			json_fetch_int_with_default   (item, "exposure_soft_min_us", (int*)&cam->ae_msv_info.exposure_soft_min_us,        (int)cam->ae_msv_info.exposure_soft_min_us);
			json_fetch_float_with_default (item, "ae_filter_alpha",      &cam->ae_msv_info.msv_filter_alpha,                  cam->ae_msv_info.msv_filter_alpha);
			json_fetch_float_with_default (item, "ae_ignore_fraction",   &cam->ae_msv_info.max_saturated_pix_ignore_fraction, cam->ae_msv_info.max_saturated_pix_ignore_fraction);
			json_fetch_float_with_default (item, "ae_slope",             &cam->ae_msv_info.exposure_gain_slope,               cam->ae_msv_info.exposure_gain_slope);
			json_fetch_int_with_default   (item, "ae_exposure_period",   (int*)&cam->ae_msv_info.exposure_update_period,      (int)cam->ae_msv_info.exposure_update_period);
			json_fetch_int_with_default   (item, "ae_gain_period",       (int*)&cam->ae_msv_info.gain_update_period,          (int)cam->ae_msv_info.gain_update_period);
		}

		// standby settings for tof only
		if(cam->type == SENSOR_TOF){
			json_fetch_bool_with_default(item, "standby_enabled", (int*)&cam->standby_enabled, cam->standby_enabled);
			json_fetch_int_with_default  (item, "decimator", &cam->decimator,   cam->decimator);
		}

		// delete some old entries
		json_remove_if_present(item, "small_video_bitrate");
		json_remove_if_present(item, "small_video_h265_en");
		json_remove_if_present(item, "large_video_bitrate");
		json_remove_if_present(item, "large_video_h265_en");
		json_remove_if_present(item, "pre_format");

	} // end of loop through cameras



	// check if we got any errors in that process
	if(json_get_parse_error_flag()){
		M_ERROR("failed to parse data in %s\n", CONFIG_FILE_NAME);
		cJSON_Delete(parent);
		return S_ERROR;
	}

	if(json_get_modified_flag()){
		if(json_write_to_file_with_header(CONFIG_FILE_NAME, parent, CONFIG_FILE_HEADER)){
			M_ERROR("failed to write config file to disk\n");
			return S_ERROR;
		}
	}

	*camera_len = numCameras;
	cJSON_free(parent);
	return S_OK;

ERROR_EXIT:

	cJSON_free(parent);
	return S_ERROR;

}

