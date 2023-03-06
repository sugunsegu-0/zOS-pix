#ifndef __GSTREAM_H__
#define __GSTREAM_H__

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <gst/gst.h>
#include <getopt.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>

struct cmdline_args {
	int width;
	int height;
	int stream;
	int s_width;
	int s_height;
	int d_width;
	int d_height;
	int no_display;
	int disable_sync;
	int record;
	int record_format;
	int record_time;
};
#define STR_PRINT(a) (a==1)?'y':'n'
#define STR_COMPARE(a,b) (strcmp(optarg,"y") == 0)?1:-1

typedef struct gst_handler {
	GstPipeline *pipeline;
	GMainLoop   *loop;
	GstElement  *src[6]; // max 6 cameras
	GstElement  *overlay;
	GstBus *bus;
	uint32_t width;
	uint32_t height;
	guint bus_watch_id;
} gst_handle_t;

struct buf_info {
	int index;
	unsigned int length;
	unsigned char *start;
	char *phy;
};

struct cap_ptr {
	guint8 *cap_ptra;
	guint8 *cap_ptrb;
	guint8 *cap_ptrc;
	guint8 *cap_ptrd;
	guint8 *cap_ptre;
	guint8 *cap_ptrf;

};

#define MAX_FILENAME 100
#define CAPTURE_MAX_BUFFER 4
#define DEF_PIPELINE_STR_LEN 1536
#define MAX_PIPELINE_STR_LEN 4096

int create_gst_handle(struct gst_handler *gst_handle, short int cameras_connected, int is_background);
void INThandler( int );
void *keyboard_event();
void *save_frame( void * );
void *jpeg_snapshot( void * );
void print_args();
void print_help( char *argv[] );
int parse_args( int argc , char *argv[] );

int get_display_resolution(struct cmdline_args *cmdline);

int gst_push_buffer(guint8 ** cap_ptr, struct gst_handler *gst_handle,
                unsigned char *framebuffer, int cameras_connected,
                int camera_num, int buffer_size);				  
void calcAspect(int *width, int *height, int *offset, int cameras_connected);
int start_stream(struct gst_handler *gst_handle);
int stream_change_state(struct gst_handler *gst_handle, int state);
int preview_frames_on_display(guint8 ** cap_ptr, struct gst_handler *gst_handle[],
                unsigned char *framebuffer, int cameras_connected, int buffer_size);
void free_gst_handle(struct gst_handler *gst_handle);
gboolean bus_message(GstBus * bus, GstMessage * message, struct gst_handler *gst_handle);

#endif
