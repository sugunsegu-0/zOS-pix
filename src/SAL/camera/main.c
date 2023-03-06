/*  tricam.elf Application
 *
 *  method used : user pointer allocation (seperate memory for each camera)
 *
 *  Image capture, with choice of displaying frames on screen given by user.
 */

#include "main.h"

#include <ecal/ecalc.h>
#include <stdio.h>
#include <string.h>

void print_help(char *argv[])
{
	fprintf(stdout, "Usage: multicam.elf [OPTION]... \n"
			" Arguments to the application: \n"
			" -w, --width=WIDTH       	width of the desired resolution \n"
			" -h, --height=HEIGHT     	height of the desired resolution \n"
			" -l, --list_res=[device] 	list the resolutions supported by the video node \n"
			" -d, --no-display=[1]    	set no-display streaming to 1 \n"
			" -s, --no-sync=[1]       	set no-sync to 1 for disable sync mode \n"
			" -r, --record=[1]        	set record to 1 for Video Recording \n"
			" -f, --record-format=[1/2] 	Available Video Recording format 1.H264 2.UYVY \n"
			"				Default Video Recording format:[H264]\n"
			" -t, --record-time=[time]	Video record time in seconds 	 \n"
			" 				Default Record time [H264=30-Sec] [UYVY=5-Sec] \n"
			" -v, --version           	prints the application version \n"
			" --help                  	prints the application usage \n"
			"Default resolution : %dx%d\n", DEFAULT__WIDTH,
			DEFAULT__HEIGHT);
	exit(1);
}

int parse_args(int argc, char *argv[])
{
	struct option long_options[] = {
		{"width", required_argument, 0, 'w'},
		{"height", required_argument, 0, 'h'},
		{"list_res", required_argument, 0, 'l'},
		{"no-display", required_argument, 0, 'd'},
		{"no-sync", required_argument, 0, 's'},
		{"record", required_argument, 0, 'r'},
		{"record-format", required_argument, 0, 'f'},
		{"record-time", required_argument, 0, 't'},
		{"version", no_argument, 0, 'v'},
		{"help", no_argument, 0, 'a'},
		{NULL, 0, NULL, 0}
	};
	int c;
	int def_w = 1;
	int def_h = 1;
	int option_index = 0;

	while ((c = getopt_long(argc, argv, "w:h:l:f:t:d:s:r:v", long_options, &option_index))) {
		if (c == -1)
			break;

		switch (c) {
			case 'w':
				def_w = 0;
				cmdline.width = atoi(optarg);
				cmdline.s_width = cmdline.width;
				break;
			case 'h':
				def_h = 0;
				cmdline.height = atoi(optarg);
				cmdline.s_height = cmdline.height;
				break;
			case 'l':
				list_resolution(optarg);
				device_data.exit_flag = 1;
				break;
			case 'f':
				cmdline.record_format = atoi(optarg);
				break;
			case 't':
				cmdline.record_time = atoi(optarg);
				break;
			case 'd':
				cmdline.no_display = atoi(optarg);
				if (cmdline.no_display != SET)
					cmdline.no_display = CLEAR;
				break;
			case 'r':
				cmdline.record = atoi(optarg);
				break;
			case 'v':
				printf("\t\tVersion: e-multicam.elf-v%s\n", VERSION);
				device_data.exit_flag = 1;
				break;
			case 's':
				cmdline.disable_sync = atoi(optarg);
				if(cmdline.disable_sync == SET)
					frame_sync_data.sync_flag = 0x00;
				else
					frame_sync_data.sync_flag = 0x3f;
				break;
			default:
				print_help(argv);
				break;
		}
	}
	if(cmdline.record == 1 && cmdline.record_format == 0)
		cmdline.record_format = 1;
	if(cmdline.record == 1 && cmdline.record_time == 0) {
		if(cmdline.record_format == 1)
			cmdline.record_time = 30;
		else
			cmdline.record_time = 5;
	}
	if(cmdline.record == 1 )
	printf(RED"################# Video Recording in Progress ... ####################\n"RESET);
	if (device_data.exit_flag != 1) {
		// Resize frames based on the size of the display.
		if (def_w != 0 || def_h != 0) {
			printf("SETTING DEFAULT RESOLUTION\n");
			cmdline.width = DEFAULT__WIDTH;
			cmdline.height = DEFAULT__HEIGHT;
			cmdline.s_width = DEFAULT__WIDTH;
			cmdline.s_height = DEFAULT__HEIGHT;
		}
		cmdline.stream = 1;
	}
	/* 
	 * Synchronous streaming not possible for 13MP and 4K Cinema Resolution
	 * Application terminated when >4K resolution provided for streaming. 
	 */
	if (cmdline.width > WIDTH_UHD || cmdline.height > HEIGHT_UHD){
		printf("RESOLUTION NOT SUPPORTED\n");
		device_data.exit_flag = 1;
	}
	return 0;
}

void print_args()
{
	printf("\nArguments set:");
	if( cmdline.width == 640 || cmdline.height == 480 ){
		printf("\nWARNING : Presently Application is Not supported for VGA 640x480 Resolution\n");
		printf("Setting to Default Resolution");
		cmdline.width = 1280;
		cmdline.height = 720;
	}
	else{
		printf("\n\twidth  : %d\n\theight : %d\n\n", cmdline.width, cmdline.height);
	}
}

void INThandler(int sig)
{
	signal(sig, SIG_IGN);
	printf(RESET "\n\t Did you hit Ctrl-C ? \n");
	printf("\t ******* EXITING ******* \n");
	device_data.exit_flag = 1;
}

int main(int argc, char *argv[])
{  

	struct app_data *app_data = NULL;
	unsigned char *fullbuffer = NULL;
	/* 2 handles : one is for stream/record and
	 * another is for background fill in stream mode
	 */
	struct gst_handler *gst_handle[2]={NULL};
	short int cam = 0, i;

	struct v4l2_buffer camera_buffer[MAX_CAM];	// v4l2 camera buffers
	guint8 *cap_ptr[MAX_CAM];

	init_app_data();

	parse_args(argc, argv);
	if (device_data.exit_flag)
		return 0;
	print_args();

	/* Identify the Jetson Device  TX1/TX2/XAVIER */
	device_data.chip_id=tegra_get_chip_id();
	if (device_data.chip_id < 0) {
		fprintf(stderr, "Unable to read CHIP ID \n");
		return -1;
	}

	app_data = (struct app_data *)malloc(sizeof(struct app_data));
	if (app_data == NULL) {
		fprintf(stderr, "appdata allocation failed\n");
		return -1;
	}

	if (init_buffer_size(cmdline)) {
		fprintf(stderr, "Failed to init_buffer_size\n");
		return -1;
	}

	/* Dynamically finding the total number of cameras connected with Jeston Camera Connector */
	get_total_camera_connected(app_data);
	if (app_data->cameras_connected == 0) {
		printf("\n\nNO CAMERA NODES PRESENT\n\n");
		free(app_data);
		app_data = NULL;
		return 0;
	}

	if (init_v4l2_camera(app_data, &cmdline, camera_buffer)) {
		fprintf(stderr, "Failed to init_v4l2_camera \n");
		free(app_data);
		return -1;
	}

	/* Updated the SYNC FLAG Value Based on How many camera connected
	 * if Frame sync enable
	 * For 1 CAM sync_flag = 0x01 (0000 0001)
	 * For 2 CAM sync_flag = 0x03 (0000 0011)
	 * For 3 CAM sync_flag = 0x07 (0000 0111)
	 * For 4 CAM sync_flag = 0x0f (0000 1111)
	 * For 5 CAM sync_flag = 0x1f (0001 1111)
	 * For 6 CAM sync_flag = 0x3f (0011 1111)
	 */
	if (frame_sync_data.sync_flag == 0x3f)
		frame_sync_data.sync_flag = stream_data.num_cam;

	if (init_gstreamer_handler( app_data, &cmdline, gst_handle)){
		fprintf(stderr, "Failed to init_gstreamer_handler \n");
		free(app_data);
		return -1;
	}

	/* Create controls thread */
	key_event_data.err = pthread_create(&key_event_data.control_tid, NULL, &init_control, (void *)app_data);
	if (key_event_data.err != 0) {
		printf("\n ERROR: Unable to create THREAD\n");
		exit(EXIT_FAILURE);
	}
	
	/* Allocate memory for  BackGround Filling in case of 2 ,3 ,5 & 6 cameras connected to the system. */
	if (Black_background(app_data)) {
		fprintf(stderr, "Failed to fill Background \n");
	}

	for (i = 0; i < CAPTURE_MAX_BUFFER; i++) {
                for (cam = 0; cam < app_data->cameras_connected ; cam++) {
                        if ((stream_data.num_cam & (1 << cam)) && (stream_data.dq_err & (1 << cam))) {
                                v4l2_capture_dq_v4l2_buffer(app_data->camera_dev[cam], camera_buffer + cam);
                                v4l2_capture_q_v4l2_buffer(app_data->camera_dev[cam], camera_buffer + cam);
                        }
                }
        }

	/* Allocate memory for Image capture buffer
	 * Memory allocation =
	 * 	(Total no. of cameras connected * save_image_len)
	 * 	Only In case of 5 camera one extra image size
	 * 	memory allocation required for image capture
	 */
	if (app_data->cameras_connected == 5) {
		fullbuffer = (unsigned char *)malloc((sizeof(unsigned char)) * 
				(img_common_data.save_image_size * (app_data->cameras_connected + 1)));
	} else {
		fullbuffer = (unsigned char *)malloc((sizeof(unsigned char)) * 
				(img_common_data.save_image_size * (app_data->cameras_connected)));
	}
	if (!fullbuffer) {
		fprintf(stderr, "malloc: %s \n", strerror(errno));
		exit(-errno);
	}

	ECAL_HANDLE pub     = 0;
  	// char        snd_s[] = "HELLO WORLD FROM C";
  	int         sent    = 0;
	eCAL_Initialize(argc, argv, "multicam", eCAL_Init_Default);
	pub = eCAL_Pub_New();
	eCAL_Pub_Create(pub, "multicam", "", "", 0);
	unsigned char* p = malloc((camera_buffer->bytesused)*(app_data->cameras_connected));
  	// eCAL_Pub_Create(pub, "Hello", "base:std::string", "", 0);
	while (key_event_data.active) {
		if (!device_data.exit_flag) {

			start_counting_frame();

			/* Get timestamp based sync frames */
			get_camera_frames(app_data, camera_buffer);
			signal(SIGINT, INThandler);
			// __u64 *p = malloc(sizeof(camera_buffer[0].bytesused * app_data->cameras_connected));
			// __u64 *p = malloc(sizeof(cap_ptr));
			
			// fprintf(stdout, "%d next is %d\n", (unsigned char *)malloc((sizeof(unsigned char)) * 
			// 	(img_common_data.save_image_size * (app_data->cameras_connected))), camera_buffer->bytesused);
			// uint8_t* p = new uint8_t[(camera_buffer[0]->bytesused)*(app_data->cameras_connected)];
			// pub.Send(camera_buffer.data(), camera_buffer->bytesused);
			for (cam = 0; cam < app_data->cameras_connected; cam++) {
				if ((stream_data.dq_err & (1 << cam))) {
					/* 
					 * Capture the frames in cap_ptr buffer.This buffer is passed around to display unit and 
					 * image capture functions. Use this buffer for any algorithms (e.g: OpenCV)
					 */
					cap_ptr[cam] = get_frame_virt(app_data, camera_buffer + cam, cam);  //sizeof cap_ptr[cam] is 8
					// printf("cap_ptr size %ld\n", sizeof(cap_ptr[cam]));
					
					// memcpy(p , cap_ptr[cam], sizeof(cap_ptr));
					memcpy((p + cam * (camera_buffer[cam].bytesused)), cap_ptr[cam], camera_buffer[cam].bytesused);
					
					// printf("%d\n", camera_buffer[cam].bytesused);
					// printf("%d\n", *cap_ptr[cam]);
					// sent = eCAL_Pub_Send(pub, cap_ptr, *cap_ptr[cam], -1);
					// if(sent <= 0) printf("Sending topic \"Hello\" failed !\n");
					// else          printf("Published topic \"Hello\" with \"\n");
				}
			}
			sent = eCAL_Pub_Send(pub, p, (camera_buffer[0].bytesused)*(app_data->cameras_connected), -1);
			// pub.Send(p, (camera_buffer->bytesused)*(app_data->cameras_connected));

			if (cmdline.no_display == SET)
				frame_rate_data.count_valid++;
			else if (cmdline.record != 1) {
				preview_frames_on_display(cap_ptr, gst_handle, stream_data.framebuffer, 
						app_data->cameras_connected, img_common_data.buffer_size);
			} else if (cmdline.record == 1 && record_data.frame_count != 0) {
				video_record(cap_ptr, gst_handle[0], app_data->cameras_connected, img_common_data.buffer_size);
				frame_rate_data.count_valid++;
			}

			/*
			 * Application terminate If recording done and exit_flag set once recording done
			 * For example:-  Record Time == 5 sec then total capture frame 150
			 */
			if ((record_data.record_count == (cmdline.record_time * DEFAULT_FRAMERATE_IN_SYNC)) && cmdline.record == 1) {
				record_data.record_count=0;
				device_data.exit_flag =1;
			}

			compute_frame_rate();

			/* Image Capture based on user input option */
			if (img_common_data.capture_mode != CAPT_MODE_DISABLE || img_common_data.writing_flag != CLEAR) 
				image_capture(stream_data.num_cam, cap_ptr, fullbuffer, img_common_data.save_image_size, cmdline,app_data);

			/* QUEUE all connected  camera buffers */
			queued_all_camera_buffers(app_data, camera_buffer);
			
			// sent = eCAL_Pub_Send(pub, snd_s, (int)strlen(snd_s), -1);
			// if(sent <= 0) printf("Sending topic \"Hello\" failed !\n");
			// else          printf("Published topic \"Hello\" with \"%s\"\n", snd_s);
			
		} else {
			/* Stop streaming and free buffers */

			stop_v4l2_streaming(app_data);

			free_application_data( app_data);	

			free_image_buffers(app_data);

			free_buffer(fullbuffer);

			free_gstreamer_buffers(app_data, gst_handle);

			free_buffer(app_data);
			return 0;
		}
	}
 	eCAL_Finalize(eCAL_Init_All);
	return 0;
}

void free_buffer(void *buffer)
{
	if (buffer != NULL) {
		free(buffer);
		buffer = NULL;
	}
}

/**
 * @brief : get capture buffer virtual address
 *
 * @param capture_buff: v4l2 capture buffer
 *
 * @return : virtual address of the captured frame
 */
guint8 *get_frame_virt(struct app_data *appdata,
		struct v4l2_buffer *capture_buff, int camera_num)
{
	return (guint8 *) appdata->camera_dev[camera_num]->buffer_pool[capture_buff->index].ptr;
}
