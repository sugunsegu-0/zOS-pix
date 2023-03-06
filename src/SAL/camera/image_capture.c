#include "main.h"
#include<jpeglib.h>

int image_capture(int num_cam, guint8 ** cap_ptr, unsigned char *fullbuffer,
		int save_image_len, struct cmdline_args cmdline, 
		struct app_data *app_data)
{
	int  cam = 0, capture_end = 0, temp = 0,camera_connected = num_cam, i;
	unsigned char *imagebuffer1=NULL, *imagebuffer2=NULL;

	/* Save image based on user input
	 * Input 6:- Capture all camera frame in single image
	 * Input 7:- Capture all camera frame in individual image
	 * 
	 * User can able to capture images in case of both SYNC and ASYNC
	 * mode but just print will displayed in case of ASYNC mode 
	 *
	 * Supported Image Capture Formats
	 * --> UYVY (RAW)
	 * --> BMP
	 * --> JPEG
	 *
	 * IF any other formats for capturing use st_image_data[cam].image_buf for UYVY RAW Data
	 */
	if ((img_common_data.capture_mode == CAPT_MODE_SINGLE || img_common_data.capture_mode == CAPT_MODE_INDIVIDUAL) 
							&& (img_common_data.writing_flag == CLEAR)) {
		if (frame_sync_data.fsyn == FRAMES_NOT_IN_SYNC && app_data->cameras_connected != 1) {
			printf("\n Frames not in sync\n");
		}
		/********************** Memory Management for Image Capture *******************
		 * In Case of upto 3 Cameras 
		 * All camera frame data copied to fullbuffer 
		 *
		 * In case of 4 to 6 cameras 
		 * 2 Extra buffer used (imagebuffer1 and imagebuffer2) for 4-6 cameras
		 *
		 * If User want any modication in final buffer or capturing in other Formats 
		 * then Use st_image_data[cam].image_buf   
		 */
		if(app_data->cameras_connected == 4 || app_data->cameras_connected == 5 
				|| app_data->cameras_connected == 6) {

			imagebuffer1 = (unsigned char *) malloc((sizeof(unsigned char)) * (img_common_data.save_image_size * 3));
			imagebuffer2 = (unsigned char *) malloc((sizeof(unsigned char)) * (img_common_data.save_image_size * 3));
			if (! imagebuffer1 || !imagebuffer2) {
				fprintf(stderr,"malloc: %s \n", strerror(errno));
				exit(-errno);
			}
			if((num_cam & (1 << 0)) && (stream_data.dq_err & (1 << 0)))  
				memcpy(imagebuffer1, ((unsigned char *) (cap_ptr[0])), img_common_data.save_image_size);
			if((num_cam & (1 << 2)) && (stream_data.dq_err & (1 << 2))) 
				memcpy(imagebuffer1 + img_common_data.save_image_size, ((unsigned char *) (cap_ptr[2])) , img_common_data.save_image_size);
			if((num_cam & (1 << 4)) && (stream_data.dq_err & (1 << 4))) 
				memcpy(imagebuffer1 + (img_common_data.save_image_size *2), ((unsigned char *) (cap_ptr[4])), img_common_data.save_image_size);
			if((num_cam & (1 << 1)) && (stream_data.dq_err & (1 << 1))) 
				memcpy(imagebuffer2 , ((unsigned char *) (cap_ptr[1])), img_common_data.save_image_size);
			if((num_cam & (1 << 3)) && (stream_data.dq_err & (1 << 3))) 
				memcpy(imagebuffer2 + img_common_data.save_image_size , ((unsigned char *) (cap_ptr[3])), img_common_data.save_image_size);
			if((num_cam & (1 << 5)) && (stream_data.dq_err & (1 << 5))) 
				memcpy(imagebuffer2 +  ((img_common_data.save_image_size * 2) ) , ((unsigned char *) (cap_ptr[5])) , img_common_data.save_image_size);
		}
		if (img_common_data.capture_mode == CAPT_MODE_SINGLE) {     //capture all frames in a single image file
			if(app_data->cameras_connected <= 3) {
				if ((num_cam & (1 << 0)) && (stream_data.dq_err & (1 << 0)))
					memcpy(fullbuffer, (unsigned char *)(cap_ptr[0]), img_common_data.save_image_size);
				if ((num_cam & (1 << 1)) && (stream_data.dq_err & (1 << 1)))
					memcpy((fullbuffer + img_common_data.save_image_size), (unsigned char *)(cap_ptr[1]), img_common_data.save_image_size);
				if ((num_cam & (1 << 2)) && (stream_data.dq_err & (1 << 2)))
					memcpy((fullbuffer + (img_common_data.save_image_size * 2)), (unsigned char *)(cap_ptr[2]), img_common_data.save_image_size);
			} else if( app_data->cameras_connected == 4) {
				for(i = 0; i < cmdline.height * 2; i++) {
					memcpy((fullbuffer + (i * cmdline.width *2*2)), (imagebuffer1 + (cmdline.width * i * 2)) 
							,(cmdline.width * 2));
					memcpy((fullbuffer + ((i * cmdline.width *2*2) + cmdline.width*2)), 
							(imagebuffer2 + (cmdline.width*i*2)),(cmdline.width*2));
				}
			}  else if(app_data->cameras_connected == 5 || app_data->cameras_connected == 6) {
				if(app_data->cameras_connected == 5) {
					memcpy(imagebuffer2 +  ((save_image_len * 2) ) , stream_data.framebuffer, img_common_data.save_image_size);
				}
				for(i = 0; i < cmdline.height * 3; i++) {
					memcpy((fullbuffer + (i * cmdline.width *2*2)), (imagebuffer1 + (cmdline.width * i * 2))
							,(cmdline.width * 2));
					memcpy((fullbuffer + ((i * cmdline.width *2*2) + cmdline.width*2)), 
							(imagebuffer2 + (cmdline.width*i*2)),(cmdline.width*2));
				}
			}

			if(st_image_data[0].image_buf != NULL) {
				free(st_image_data[0].image_buf);
				st_image_data[0].image_buf = NULL;
			}

			if(app_data->cameras_connected == 5) {	
				st_image_data[0].image_buf = (unsigned char *)malloc((sizeof(unsigned char)) * (img_common_data.save_image_size * (app_data->cameras_connected + 1)));
				memcpy(st_image_data[0].image_buf, fullbuffer, img_common_data.save_image_size * (app_data->cameras_connected + 1));
			} else {
				st_image_data[0].image_buf = (unsigned char *)malloc((sizeof(unsigned char)) * (img_common_data.save_image_size * app_data->cameras_connected));
				memcpy(st_image_data[0].image_buf, fullbuffer, img_common_data.save_image_size * (app_data->cameras_connected));
			}
			st_image_data[0].width = cmdline.width;
			st_image_data[0].height = cmdline.height;
			st_image_data[0].cam_num = app_data->cameras_connected;

			key_event_data.err = write_image(&st_image_data[0]);
			if (key_event_data.err != 0) {
				printf("\n ERROR: Unable to capture Image\n");
				exit(EXIT_FAILURE);
			}

		} else if (img_common_data.capture_mode == CAPT_MODE_INDIVIDUAL) {  //capture frames in individual image files
			for (cam = 0; cam < app_data->cameras_connected; cam++) {
				if ((num_cam & (1 << cam))) {
					camera_connected = camera_connected ^ (1 << temp++);
					if (camera_connected == 0) {
						capture_end = 1;
						temp = 0;
						camera_connected = num_cam;
					}

					if(st_image_data[cam].image_buf != NULL) {
						free(st_image_data[cam].image_buf);
						st_image_data[cam].image_buf = NULL;
					}

					st_image_data[cam].image_buf = (unsigned char *)
						malloc((sizeof(unsigned char)) * (img_common_data.save_image_size));
					memcpy(st_image_data[cam].image_buf, (unsigned int *)(cap_ptr[cam]), img_common_data.save_image_size);
					st_image_data[cam].width = cmdline.width;
					st_image_data[cam].height = cmdline.height;
					st_image_data[cam].cam_num = cam;
					st_image_data[cam].captr_end = capture_end;
					capture_end = 0;

					
					key_event_data.err = write_image(&st_image_data[cam]);
					if (key_event_data.err != 0) {
						printf("\n ERROR: Unable to capture Images\n");
						exit(EXIT_FAILURE);
					}
				}
			}
		}
	}
	/* Clear the image capture flag for once again capturing */
	if (img_common_data.capture_mode == CAPT_MODE_DISABLE && img_common_data.image_capture_flag > CLEAR) {
		if (img_common_data.image_capture_flag == SINGLE_IMAGE_CAPTURE) {
			img_common_data.image_capture_flag = CLEAR;
			printf("\nImage captured.. \n");
			if(app_data->cameras_connected == 4 || 
					app_data->cameras_connected == 5 || 
					app_data->cameras_connected == 6) {
				free(imagebuffer1);
				free(imagebuffer2);
			} 
		} else {
			img_common_data.image_capture_flag = CLEAR;
			printf("\nImage captured.. \n");
		}
		img_common_data.writing_flag = CLEAR;
	}
	return 0;
}

/*write image data to the File*/
int write_image(struct image_data *image)
{
	int width, height;
	unsigned long int size;
	if (img_common_data.capture_mode == CAPT_MODE_SINGLE || img_common_data.capture_mode == CAPT_MODE_INDIVIDUAL) {
		if (img_common_data.writing_flag != SET && img_common_data.capture_mode != CAPT_MODE_INDIVIDUAL) {
			img_common_data.writing_flag = SET;
			if( image->cam_num <= 3) {
				width = image->width  ;
				height = image->height * image->cam_num;
			} else if(image->cam_num == 4) {
				width = image->width * 2;
				height = image->height * 2;
			} else if(image->cam_num == 5 || image->cam_num == 6) {
				width = image->width * 2;
				height = image->height * 3;
			}
			size = width * height * UYVY_BYTES_PER_PIXEL;
			switch (img_common_data.img_format) {
				case IMG_FMT_RAW:
					save_raw_image(image->image_buf, width, height, size, image->cam_num, image->captr_end);
					break;
				case IMG_FMT_BMP:
					save_bmp_image(image->image_buf, width, height, size, image->cam_num, image->captr_end);
					break;
				case IMG_FMT_JPG:
					write_jpeg_file(image->image_buf, width, height, image->cam_num, image->captr_end, size);
					break;
			}
		} else if (img_common_data.capture_mode == CAPT_MODE_INDIVIDUAL) {
			width = image->width;
			height = image->height;
			size = width * height * UYVY_BYTES_PER_PIXEL;

			switch (img_common_data.img_format) {
				case IMG_FMT_RAW:
					save_raw_image(image->image_buf, width, height, size, image->cam_num, image->captr_end);
					break;
				case IMG_FMT_BMP:
					save_bmp_image(image->image_buf, width, height, size, image->cam_num, image->captr_end);
					break;
				case IMG_FMT_JPG:
					write_jpeg_file(image->image_buf, width, height, image->cam_num, image->captr_end, size);
					break;
			}
		}
	}
	if (image->image_buf) {
		free(image->image_buf);
		image->image_buf = NULL;
	}
	return 0;
}

int yuv_rgb888_conversion(unsigned char *yuyv_buffer, unsigned char *rgb_buffer, int width_act, int height_act)
{
    int i = 0, j = 0;
    int temp;
    int yuvcount = 0;
    int rgbcount = 0;
    int width = width_act;
    int height = height_act;
    float u_val, v_val, y1_val, y2_val;

    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j += 2) {
            u_val = (float)yuyv_buffer[yuvcount++];
            y1_val = (float)yuyv_buffer[yuvcount++];
            v_val = (float)yuyv_buffer[yuvcount++];
            y2_val = (float)yuyv_buffer[yuvcount++];

            u_val = u_val - 128;
            v_val = v_val - 128;

            temp = (int)(y1_val + (1.770 * u_val));
            rgb_buffer[(((height - 1) - i) * width * 3) + j * 3 + 0] =
                (temp > 255) ? 255 : ((temp < 0) ? 0 : (unsigned char)temp);
            rgbcount++;

            temp = (int)(y1_val - (0.344 * u_val) - (0.714 * v_val));
            rgb_buffer[(((height - 1) - i) * width * 3) + j * 3 + 1] =
                (temp > 255) ? 255 : ((temp < 0) ? 0 : (unsigned char)temp);
            rgbcount++;

            temp = (int)(y1_val + (1.403 * v_val));
            rgb_buffer[(((height - 1) - i) * width * 3) + j * 3 + 2] =
                (temp > 255) ? 255 : ((temp < 0) ? 0 : (unsigned char)temp);
            rgbcount++;

            temp = (int)(y2_val + (1.770 * u_val));
            rgb_buffer[(((height - 1) - i) * width * 3) + j * 3 + 3] =
                (temp > 255) ? 255 : ((temp < 0) ? 0 : (unsigned char)temp);
            rgbcount++;

            temp = (int)(y2_val - (0.344 * u_val) - (0.714 * v_val));
            rgb_buffer[(((height - 1) - i) * width * 3) + j * 3 + 4] =
                (temp > 255) ? 255 : ((temp < 0) ? 0 : (unsigned char)temp);
            rgbcount++;

            temp = (int)(y2_val + (1.403 * v_val));
            rgb_buffer[(((height - 1) - i) * width * 3) + j * 3 + 5] =
                (temp > 255) ? 255 : ((temp < 0) ? 0 : (unsigned char)temp);
            rgbcount++;
        }
    }
    return 0;
}

void save_bmp_image(unsigned char *fullbuffer, int width, int height, unsigned long int size, int num, int captr_end)
{
    int fd = 0;
    unsigned char *rgb888 = malloc((size * RGB888_BYTES_PER_PIXEL) / 2);

    img_common_data.writing_flag = SET;
    time_t sys_time = time(NULL);
    struct tm img_time = *localtime(&sys_time);
    char *filename = NULL;
    char *homedir = getenv("HOME");
    filename = (char *) malloc(FILE_NAME_SIZE * sizeof(char));
    sprintf(filename,"image-%d-%d-%d_%d.bmp",img_time.tm_hour,img_time.tm_min,img_time.tm_sec,num);
    filename = g_strjoin ( "/", homedir ,filename, NULL);

    struct __attribute__ ((__packed__)) {
        unsigned short type;
        unsigned int size;
        unsigned short reserved1;
        unsigned short reserved2;
        unsigned int offbits;
        unsigned int size_header;
        int width;
        int height;
        unsigned short planes;
        unsigned short bitcount;
        unsigned int compression;
        unsigned int size_image;
        int x_permeter;
        int y_permeter;
        unsigned int clr_used;
        unsigned int clr_important;
    } bmp_header = {
    	.type = 0x4D42,
		.size = sizeof(bmp_header) + ((size * RGB888_BYTES_PER_PIXEL) / 2),
		.reserved1 = 0,
		.reserved2 = 0,
		.offbits = sizeof(bmp_header),
		.size_header = 40,
		.width = width,
		.height = height,
		.planes = 1,
		.bitcount = 24,
		.compression = 0,
		.size_image = ((size * RGB888_BYTES_PER_PIXEL) / 2),
		.x_permeter = 0x0B13,
		.y_permeter = 0x0B13,
		.clr_used = 0,
		.clr_important = 0,
	};
    if (fd > 0)
        close(fd);
    fd = open(filename, O_WRONLY | O_CREAT | O_SYNC, 0666);
    if (fd < 0) {
        printf("Save failed time stamp %s %s %d\n", __FILE__, __func__, __LINE__);
    }
    yuv_rgb888_conversion(fullbuffer, rgb888, width, height);
    write(fd, &bmp_header, sizeof(bmp_header));
    write(fd, rgb888, ((size * RGB888_BYTES_PER_PIXEL) / 2));
    free(rgb888);
    close(fd);
    printf("File saved: %s \n", filename);

    if (captr_end == 1) {
        img_common_data.image_capture_flag = INDIVIDUAL_IMAGE_CAPTURE;
        img_common_data.capture_mode = CAPT_MODE_DISABLE;
    } else if (img_common_data.capture_mode == CAPT_MODE_SINGLE) {
        img_common_data.image_capture_flag = SINGLE_IMAGE_CAPTURE;
        img_common_data.capture_mode = CAPT_MODE_DISABLE;
    }
    free(filename);
    return;
}

void save_raw_image(unsigned char *fullbuffer, int width, int height, unsigned long int size, int num, int captr_end)
{
    img_common_data.writing_flag = SET;
    time_t sys_time = time(NULL);
    struct tm img_time = *localtime(&sys_time);
    char *filename = NULL;
    filename = (char *) malloc(FILE_NAME_SIZE * sizeof(char));
    char *homedir = getenv("HOME");
    sprintf(filename,"image-%d-%d-%d_%d.yuv",img_time.tm_hour,img_time.tm_min,img_time.tm_sec,num);
    filename = g_strjoin ( "/", homedir ,filename, NULL);
    int fp = open(filename, O_WRONLY | O_CREAT, 0666);
    if (fp < 0) {
        fprintf(stderr, "file open: %s\n", strerror(errno));
        free(filename);
        exit(0);
    }
    
    write(fp, fullbuffer, size);
    close(fp);
    printf("File saved: %s \n", filename);
    
    
    if (captr_end == 1) {
        img_common_data.image_capture_flag = INDIVIDUAL_IMAGE_CAPTURE;
        img_common_data.capture_mode = CAPT_MODE_DISABLE;
    } else if (img_common_data.capture_mode == CAPT_MODE_SINGLE) {
        img_common_data.image_capture_flag = SINGLE_IMAGE_CAPTURE;
        img_common_data.capture_mode = CAPT_MODE_DISABLE;
    }
    free(filename);
}

int write_jpeg_file(unsigned char *image_data, int width, int height, int num, int captr_end , int image_size )
{
	img_common_data.writing_flag = SET;
        time_t sys_time = time(NULL);
        struct tm img_time = *localtime(&sys_time);
        char *filename = NULL;
	char *homedir = getenv("HOME");
        filename = (char *) malloc(FILE_NAME_SIZE * sizeof(char));
	sprintf(filename,"image-%d-%d-%d_%d.jpg",img_time.tm_hour,img_time.tm_min,img_time.tm_sec,num);
    	filename = g_strjoin ( "/", homedir ,filename, NULL);
    	FILE *fp = fopen(filename, "wb" );
    	if (fp == NULL) {
        	fprintf(stderr, "file open: %s\n", strerror(errno));
                printf("Error opening output jpeg file %s\n!", filename );
        	free(filename);
        	exit(0);
    	}

        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;
        int bytes_per_pixel;
        int color_space ;  /* JCS_YCbCr for UYVY images */


        JSAMPROW row_pointer[1];
        unsigned char *tmprowbuf;
        tmprowbuf = (unsigned char *) malloc(width * 3 * sizeof(char));
        bytes_per_pixel = 3;
        color_space = JCS_YCbCr; //JCS_RGB //JCS_CMYK /* C/M/Y/K */  //JCS_YCCK,

        cinfo.err = jpeg_std_error( &jerr );
        jpeg_create_compress(&cinfo);
        jpeg_stdio_dest(&cinfo, fp);

        cinfo.image_width = width;
        cinfo.image_height = height;
        cinfo.input_components = bytes_per_pixel;
        cinfo.in_color_space = color_space;
        cinfo.data_precision = 8; // bits_per_pixel;
        cinfo.input_gamma = 1.00;
        jpeg_set_defaults( &cinfo );
        jpeg_set_quality(&cinfo, 75, TRUE);

        jpeg_start_compress( &cinfo, TRUE );
        row_pointer[0] = &tmprowbuf[0];

        while( cinfo.next_scanline < cinfo.image_height )
        {
                unsigned i, j;
                unsigned offset = cinfo.next_scanline * cinfo.image_width * 2;
                for (i = 0, j = 0; i < cinfo.image_width * 2; i += 4, j += 6) {
                        tmprowbuf[j + 0] = image_data[offset + i + 1]; // U 
                        tmprowbuf[j + 1] = image_data[offset + i + 0]; // Y 
                        tmprowbuf[j + 2] = image_data[offset + i + 2]; // V
                        tmprowbuf[j + 3] = image_data[offset + i + 3]; // Y
                        tmprowbuf[j + 4] = image_data[offset + i + 0]; // U
                        tmprowbuf[j + 5] = image_data[offset + i + 2]; // V
                }
                jpeg_write_scanlines(&cinfo, row_pointer, 1);
        }

        if (captr_end == 1) {
                img_common_data.image_capture_flag = INDIVIDUAL_IMAGE_CAPTURE;
                img_common_data.capture_mode = CAPT_MODE_DISABLE;
        } else if (img_common_data.capture_mode == CAPT_MODE_SINGLE) {
                img_common_data.image_capture_flag = SINGLE_IMAGE_CAPTURE;
                img_common_data.capture_mode = CAPT_MODE_DISABLE;
        }
        jpeg_finish_compress( &cinfo );
        jpeg_destroy_compress( &cinfo );
	fclose(fp);
    	printf("File saved: %s \n", filename);
	free(filename);
        free(tmprowbuf);

        return 0;
}
