
#include "main.h"
#include "gstream.h"

int create_record_handle(struct gst_handler *gst_handle, short int cameras_connected)
{
	char pipeline[MAX_PIPELINE_STR_LEN];
	int cam_id;
	time_t sys_time = time(NULL);
	struct tm record_time = *localtime(&sys_time);
	char *homedir = getenv("HOME");
	char *filename = NULL;

	pipeline[0] = '\0';

	filename = (char *) malloc(FILE_NAME_SIZE * sizeof(char));

	gst_init(NULL, NULL);

	/******************** GSTREAMER PIPELINE DETAILS **********************
	 * SAVE_H264_VIDEO_PIPELINE for H264 video Recording
	 * SAVE_UYVY_VIDEO_PIPELINE for UVYV video Recording
	 *
	 * Recording  pipeline for H264:-
	 * [appsrc] ---> [nvvidconv] --> [nvv4l2h264enc] --> [h264parse] --> [matroskamux] --> [filesink]
	 * Recording  pipeline for UYVY:-
	 * [appsrc]  --> [filesink]
	 *
	 * NOTE:- 
	 * 	1. If user want to record video in VP8/VP9/H265 then Just change the pipeline 
	 * 	2. Be careful with UYVY video recording because for 1080p recording just for 
	 * 	   5 sec takes around ~500MB memory for 1 camera. 
	 */


	for (cam_id = 0; cam_id < cameras_connected; cam_id++) {
		char temp[DEF_PIPELINE_STR_LEN];
		
		if (cmdline.record_format == 1) { 
			sprintf(filename, "Videos/video-%d-%d-%d_%d.mkv", record_time.tm_hour, 
					record_time.tm_min, record_time.tm_sec, cam_id); 
			filename = g_strjoin("/", homedir, filename, NULL);
			sprintf(temp, SAVE_H264_VIDEO_PIPELINE, cam_id, cmdline.width, cmdline.height, cmdline.width, cmdline.height, filename);
		} else if (cmdline.record_format == 2) {
			sprintf(filename, "Videos/video-%d-%d-%d_%d.uyvy", record_time.tm_hour, 
					record_time.tm_min, record_time.tm_sec, cam_id); 
			filename = g_strjoin("/", homedir, filename, NULL);
			sprintf(temp, SAVE_UYVY_VIDEO_PIPELINE, cam_id, cmdline.width, cmdline.height, filename);
		}

		strcat(pipeline, temp);
	}

	gst_handle->pipeline = (GstPipeline *) gst_parse_launch(pipeline, NULL);
	/*      Get referance to the gst bus from pipeline      */
	gst_handle->bus = gst_pipeline_get_bus(gst_handle->pipeline);
	gst_handle->bus_watch_id = gst_bus_add_watch(gst_handle->bus, (GstBusFunc) bus_message, gst_handle);
	gst_object_unref(gst_handle->bus);

	/* set the caps on the source */
	/*fmt,width,height,fps_n,fps_:d,asp_n,asp_d */
	gst_handle->width = cmdline.width;
	gst_handle->height = cmdline.height;
	for (cam_id = 0; cam_id < cameras_connected; cam_id++) {
		char cam_src[16];
		sprintf(cam_src, "mysource%d", cam_id);
		gst_handle->src[cam_id] = gst_bin_get_by_name(GST_BIN(gst_handle->pipeline), cam_src);

		g_object_set(G_OBJECT(gst_handle->src[cam_id]), "caps",
				gst_caps_new_simple("video/x-raw",
					"format", G_TYPE_STRING, "UYVY",
					"width", G_TYPE_INT,
					cmdline.width, "height",
					G_TYPE_INT,
					cmdline.height,
					"framerate", GST_TYPE_FRACTION, 30, 1, NULL), NULL);
		g_object_set(gst_handle->src[cam_id], "do-timestamp", TRUE, NULL);
		g_object_set(G_OBJECT(gst_handle->src[cam_id]), "block", TRUE, NULL);
		//g_object_set(G_OBJECT(gst_handle->src[cam_id]), "format", GST_FORMAT_TIME, NULL);
	}

	gst_handle->loop = g_main_loop_new(NULL, FALSE);
	free(filename);
	return 0;
}

int video_record(guint8 ** cap_ptr, struct gst_handler *gst_handle,
		int cameras_connected, int buffer_size)
{
	GstBuffer *buffer = NULL;
	GstFlowReturn ret = TRUE;
	int cam_id, no_error = 1;

	for (cam_id = 0; cam_id < cameras_connected; cam_id++) {
		buffer = gst_buffer_new_allocate(NULL, buffer_size, NULL);
		gst_buffer_fill(buffer, 0, (gpointer) ((unsigned char *)(cap_ptr[cam_id])), buffer_size);

		g_signal_emit_by_name(gst_handle->src[cam_id], "push-buffer", buffer, &ret);
		if (ret != GST_FLOW_OK) {
			/* some error, stop sending data */
			fprintf(stderr, "some error1\n");
			no_error = 0;
		}
		gst_buffer_unref(buffer);
	}

	return no_error;
}
