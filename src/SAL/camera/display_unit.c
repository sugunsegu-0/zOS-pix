#include "main.h"
#include "display_unit.h"

gboolean bus_message(GstBus * bus, GstMessage * message, struct gst_handler *gst_handle);

void fill_display_background(void *dest, unsigned int size)
{
       unsigned int *buf = dest;
       unsigned int i;
       for (i = 0; i < size / (sizeof(unsigned int)); i++) {
               *(buf + i) =   0x00800080;
       }
}

int get_display_resolution(struct cmdline_args *cmdline)
{
#if 0
	int disp_fd;
	struct fb_var_screeninfo  disp_info;

	disp_fd = open(DISPLAY_DEVNAME_DEFAULT, O_RDONLY, 0);
	if(disp_fd < 0) {
		fprintf(stderr, "Can't open display device \n");
		return -1;
	}

	if(ioctl(disp_fd, FBIOGET_VSCREENINFO, &disp_info) < 0) {
		perror("DISPLAY: FBIOPUT_VSCREENINFO");
		return -1;
	}
#endif
	//FIXME not reading display resolution properly
	cmdline->d_width = 1920;//disp_info.xres;
	cmdline->d_height = 1080;//disp_info.yres;

	//close(disp_fd);
	return 0;
}


int preview_frames_on_display(guint8 ** cap_ptr, struct gst_handler *gst_handle[],
		unsigned char *framebuffer, int cameras_connected, int buffer_size)
{
	int cam_id = 0;
	static int fill_background = 1;

	/*
	 * As nvoverlaysink was deprecated, using nvcompositor and nvdrmvideosink.
	 * [note: there is video display issues when using only nvdrmvideosink 
	 *        with overlay(planes) support for multiple streamings]
	 * 
	 * Filling Display Background common for all cameras
	 */
	for (cam_id = 0; cam_id < cameras_connected; cam_id++)
		gst_push_buffer(cap_ptr, gst_handle[0], framebuffer, cameras_connected, cam_id, img_common_data.buffer_size);

	if (fill_background) {
		gst_push_buffer(cap_ptr, gst_handle[1], framebuffer, cameras_connected, cam_id, stream_data.display_size);
		fill_background = 0;
	}

	frame_rate_data.count_valid++;
	return 0;
}

int gst_push_buffer(guint8 ** cap_ptr, struct gst_handler *gst_handle,
		unsigned char *framebuffer, int cameras_connected, 
		int cam_id, int buffer_size)
{
	GstBuffer *buffer = NULL;   // for preview
	GstFlowReturn ret = TRUE;

	buffer = gst_buffer_new_allocate(NULL, buffer_size, NULL);

	/* when cam_id is equal to total no of cameras,
	 * it is for background
	 */
	if ((cameras_connected == cam_id)) {
		gst_buffer_fill(buffer, 0, (unsigned char *) framebuffer, buffer_size);
		g_signal_emit_by_name(gst_handle->src[0], "push-buffer", buffer, &ret);
	} else {
		gst_buffer_fill(buffer, 0, (gpointer) ((unsigned char *)(cap_ptr[cam_id])), buffer_size);
		g_signal_emit_by_name(gst_handle->src[cam_id], "push-buffer", buffer, &ret);
	}

	if (ret != GST_FLOW_OK) {
		/* some error, stop sending data */
		fprintf(stderr, "some error1\n");
	}

	gst_buffer_unref(buffer);

	return (ret == GST_FLOW_OK) ? 1 : 0;
}

void calcAspect(int *width, int *height, int *offset, int cameras_connected)
{
	if(cameras_connected < 4) 
		*height = cmdline.d_height / cameras_connected;
	else if(cameras_connected == 4)
		*height = cmdline.d_height / 2;
	else if(cameras_connected == 5 || cameras_connected == 6)
		*height = cmdline.d_height / 3;

	*width = (int) round(((float) *height * 16) / 9);
	switch (cameras_connected) {	
		case 1:
		case 4:
			*offset = 0;
			break;
		case 2: *offset = cmdline.d_width / 4;
			break;
		case 3: *offset = cmdline.d_width / 3;
			break;
		case 5:
		case 6:
			*offset = 360;
		default:
			break;
	}
	/* Handle case for HD+ (1600x900) display. */
	if ( *width % 2 != 0 ) 
		*width  = *width-1;
}

int create_gst_handle(struct gst_handler *gst_handle, short int cameras_connected, int is_background)
{
	char pipeline[MAX_PIPELINE_STR_LEN];
	char pipeline_1[DEF_PIPELINE_STR_LEN];
	char pipeline_2[DEF_PIPELINE_STR_LEN];
	int aspect_height, overlay_offset = 0, aspect_width;
	int x_pos=0, y_pos=0;
	int overlay_flag = 5; /* 4 is for background and 5 is for streaming */
	int cam_id = 0;

	pipeline_1[0] = '\0';
	pipeline_2[0] = '\0';
	pipeline[0] = '\0';
	gst_init(NULL, NULL);

	/**************** Nvidia Overlay Usage ***************
	 * NUM CAM = 1 Overlay_num = {1}
	 * NUM CAM = 2 Overlay_num = {1,2,3} 3rd overlay for Background filling
	 * NUM CAM = 3 Overlay_num = {1,2,3,4} 4th overlay for Background filling
	 * NUM CAM = 4 Overlay_num = {1,2,3,4}
	 * NUM CAM = 5 Overlay_num = {1,2,3,4,5} 5th overlay for background
	 * NUM CAM = 6 Overlay_num = {1,2,3,4,5} 5th overlay used for Background
	 */
	/* As nvoverlaysink was deprecated, using nvcompositor and nvdrmvideosink.
	 * [note: there is video display issues when using only nvdrmvideosink 
	 *        with overlay(planes) support for multiple streamings]
	 * Plane/Overlay 5 is used for multiple video display.
	 * Plane/Overlay 4 is used for background.
	 */

	device_data.overlay_num = overlay_flag - is_background;

	/* Aspect Ratio calculation for Proper frame display based on display aspect ration */
	calcAspect(&aspect_width, &aspect_height, &overlay_offset, cameras_connected);

	if (!is_background) { // camera streaming
		/**************** X and Y positions for ALL Camera combination ******************************************************
		 *0 NUM_CAMERA = 1 X = 0  Y = 0  									(offset = 0)
		 *1 NUM_CAMERA = 2 X1 =0 Y1 = 0  X2=0 Y2=540 								(offset = 480)
		 *2 NUM_CAMERA = 3 X1 =0 Y1 = 0, X2=0 Y2=360, X3=0  Y3=1440  						(offset = 640)
		 *3 NUM_CAMERA = 4 X1 =0 Y1 = 0, X2=960 Y2=0, X3=0 Y3=540, X4=960 Y4=540 				(offset = 0  ) 
		 *4 NUM_CAMERA = 5 X1 =0 Y1 = 0, X2=640 Y2=0, X3=0 Y3=360, X4=640 Y4=360, X5=0 Y5=720 			(offset = 480
		 *5 NUM_CAMERA = 6 X1 =0 Y1 = 0, X2=640 Y2=0, X3=0 Y3=360, X4=640 Y4=360, X5=0 Y5=720, X6=640 Y6=720	(offset = 360) 
		 */
		for (cam_id = 0; cam_id < cameras_connected; cam_id++) {
			char temp[DEF_PIPELINE_STR_LEN];

			if (cameras_connected <= 3) {	
				x_pos = overlay_offset;
				y_pos = aspect_height * cam_id;
			} else if (cameras_connected == 4 || cameras_connected == 5  || cameras_connected == 6) {
				if (cam_id == CAMERA_1) {
					x_pos = overlay_offset + (aspect_width * cam_id);
					y_pos =  aspect_height * cam_id;
				} else if (cam_id == CAMERA_2) {
					x_pos = overlay_offset + (aspect_width * cam_id);
					y_pos =  aspect_height * (cam_id - 1);
				} else if (cam_id == CAMERA_3) {
					x_pos = overlay_offset + (aspect_width * (cam_id - 2));
					y_pos =  aspect_height * (cam_id - 1) ;
				} else if (cam_id == CAMERA_4) {
					x_pos = overlay_offset + aspect_width;
					y_pos =  aspect_height;
				} else if (cam_id == CAMERA_5) {
					x_pos = overlay_offset + (aspect_width * (cam_id - 4));
					y_pos =  aspect_height * 2;
				} else if (cam_id == CAMERA_6) {
					x_pos = overlay_offset + aspect_width;
					y_pos =  aspect_height * 2;
				}
			}

			/******************** GSTREAMER PIPELINE DETAILS **********************
			 * STREAM_PIPELINE for Live Camera preview 
			 *
			 * Streaming  pipeline:-
			 * [appsrc] ---> [nvvidconv] --> [nvdrmvideosink] --> [nvcompositor]
			 */

			sprintf(temp, STREAM_PIPELINE_1, cam_id, x_pos, cam_id, y_pos);
			strcat(pipeline_1, temp);
			sprintf(temp, STREAM_PIPELINE_2, cam_id, cmdline.width, 
					cmdline.height, aspect_width, aspect_height); 
			strcat(pipeline_2, temp);
		}

		sprintf(pipeline, STREAM_PIPELINE_1_PRE"%s"STREAM_PIPELINE_1_POST"%s", pipeline_1, device_data.overlay_num, pipeline_2);

		gst_handle->pipeline = (GstPipeline *) gst_parse_launch(pipeline, NULL);
		/*	Get referance to the gst bus from pipeline	*/
		gst_handle->bus = gst_pipeline_get_bus(gst_handle->pipeline);
		gst_handle->bus_watch_id = gst_bus_add_watch(gst_handle->bus, (GstBusFunc) bus_message, gst_handle);
		gst_object_unref(gst_handle->bus);

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
			g_object_set(G_OBJECT(gst_handle->src[cam_id]), "is-live", TRUE, NULL);
			g_object_set(G_OBJECT(gst_handle->src[cam_id]), "do-timestamp", TRUE, NULL);
			g_object_set(G_OBJECT(gst_handle->src[cam_id]), "block", TRUE, NULL);
			g_object_set(G_OBJECT(gst_handle->src[cam_id]), "format", GST_FORMAT_TIME, NULL);
		}
	} else { // background
		/******************** GSTREAMER PIPELINE DETAILS **********************
		 * BACKGROUND_STREAM_PIPELINE for Preview Background filling
		 */
		sprintf(pipeline, BACKGROUND_STREAM_PIPELINE, cmdline.d_width, cmdline.d_height,
				cmdline.d_width, cmdline.d_height, device_data.overlay_num);
		gst_handle->pipeline = (GstPipeline *) gst_parse_launch(pipeline, NULL);
		/*	Get referance to the gst bus from pipeline	*/
		gst_handle->bus = gst_pipeline_get_bus(gst_handle->pipeline);
		gst_handle->bus_watch_id = gst_bus_add_watch(gst_handle->bus, (GstBusFunc) bus_message, gst_handle);
		gst_object_unref(gst_handle->bus);
		gst_handle->src[0] = gst_bin_get_by_name(GST_BIN(gst_handle->pipeline), "mysource");
		gst_handle->width = cmdline.width;
		gst_handle->height = cmdline.height;
		g_object_set(G_OBJECT(gst_handle->src[0]), "caps",
				gst_caps_new_simple("video/x-raw",
					"format", G_TYPE_STRING, "UYVY",
					"width", G_TYPE_INT,
					cmdline.d_width, "height",
					G_TYPE_INT,
					cmdline.d_height,
					"framerate", GST_TYPE_FRACTION, 30, 1, NULL), NULL);
		g_object_set(G_OBJECT(gst_handle->src[0]), "is-live", TRUE, NULL);
		g_object_set(G_OBJECT(gst_handle->src[0]), "do-timestamp", TRUE, NULL);
		g_object_set(G_OBJECT(gst_handle->src[0]), "block", TRUE, NULL);
		g_object_set(G_OBJECT(gst_handle->src[0]), "format", GST_FORMAT_TIME, NULL);
	}

	gst_handle->loop = g_main_loop_new(NULL, FALSE);

	return 0;
}

int start_stream(struct gst_handler *gst_handle)
{
	/* Changing Pipeline state to GST_STATE_PLAYING */
	return stream_change_state(gst_handle, GST_STATE_PLAYING);
}

int stream_change_state(struct gst_handler *gst_handle, int state)
{
	GstStateChangeReturn ret;
	ret = gst_element_set_state((GstElement *) gst_handle->pipeline, state);

	if (ret == GST_STATE_CHANGE_FAILURE) {
		g_printerr("Unable to set the pipeline to playing state\n");
		printf("Unable to set the pipeline to playing state\n");
		gst_object_unref(gst_handle->pipeline);
		return -1;
	}
	return 0;
}

/**
 * @brief : This callback let you know about any error / event happening 
 * int the gstreamer pipeline.
 *
 * @param bus: gstreamer pipeline bus
 * @param msg: gstreamer pipeline messages
 * @param data: user defined data for the callback
 *
 * @return: ture on success 
 */
gboolean bus_message(GstBus * bus, GstMessage * message, struct gst_handler *gst_handle)
{
	GST_DEBUG("got message %s", gst_message_type_get_name(GST_MESSAGE_TYPE(message)));

	switch (GST_MESSAGE_TYPE(message)) {
		case GST_MESSAGE_ERROR:
			g_error("received error");
			g_main_loop_quit(gst_handle->loop);
			break;
		case GST_MESSAGE_EOS:
			g_main_loop_quit(gst_handle->loop);
			break;
		default:
			break;
	}
	return TRUE;
}

void free_gst_handle(struct gst_handler *gst_handle)
{
	gst_element_set_state((GstElement *) gst_handle->pipeline, GST_STATE_NULL);

	gst_object_unref(gst_handle->bus);

	g_source_remove(gst_handle->bus_watch_id);

	g_main_loop_unref(gst_handle->loop);

	if (gst_handle != NULL)
		free(gst_handle);
}
