

#define SAVE_H264_VIDEO_PIPELINE	"appsrc name=mysource%d !" \
	" video/x-raw,format=(string)UYVY,width=%d,height=%d !" \
	" nvvidconv ! video/x-raw(memory:NVMM),width=%d,height=%d,format=I420 !" \
	" nvv4l2h264enc qp-range=20,30:20,30:-1,-1 ! h264parse !" \
	" video/x-h264,stream-format=(string)avc,alignment=(string)au ! matroskamux !" \
	" video/x-matroska ! filesink location=%s "

#define  SAVE_UYVY_VIDEO_PIPELINE     	"appsrc name=mysource%d !" \
	" video/x-raw,format=(string)UYVY,width=%d,height=%d ! filesink location=%s "

struct record_video {
	int record_count;
	int frame_count;
}record_data;

int video_record(guint8 ** cap_ptr, struct gst_handler *gst_handle,
		int cameras_connected, int buffer_size);
int create_record_handle(struct gst_handler *gst_handle, short int cameras_connected);
