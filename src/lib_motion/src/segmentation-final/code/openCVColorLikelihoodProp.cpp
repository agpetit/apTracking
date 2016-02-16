/* --Sparse Optical Flow Demo Program--
 * Written by David Stavens (david.stavens@ai.stanford.edu)
 */
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>

#include "mex.h"
#include "matrix.h"

static const double pi = 3.14159265358979323846;

inline static double square(int a)
{
	return a * a;
}

/* This is just an inline that allocates images.  I did this to reduce clutter in the
 * actual computer vision algorithmic code.  Basically it allocates the requested image
 * unless that image is already non-NULL.  It always leaves a non-NULL image as-is even
 * if that image's size, depth, and/or channels are different than the request.
 */
//IplImage: the data structure that contains the image under OpenCV
//CvSize: pixel-accurate size of a rectangle, including width and height
inline static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels )
{
	if ( *img != NULL )	return;

	*img = cvCreateImage( size, depth, channels );  //create an image header and allocates the image data
                                        //size: image width and height; depth: bit depth; channels: number of channels per pixel
	if ( *img == NULL )
	{
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}

#define MAX_NUM_OF_FEATURES 400
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
	/* Read the video's frame size out of the AVI. */
	CvSize frame_size;
	frame_size.height = (int) mxGetScalar(prhs[0]);
	frame_size.width = (int) mxGetScalar(prhs[1]);
    unsigned char* mask;
    long current_frame = (long) mxGetScalar(prhs[2]);
    mask = (unsigned char*) mxGetData(prhs[3]);
	int fPathLen = (mxGetM(prhs[4]) * mxGetN(prhs[4])) + 1;
	char* fPath = mxArrayToString(prhs[4]);
    /*FILE* log;
    log = fopen("opencvMask.txt", "w");
    for (int i = 0; i < frame_size.height; ++i) {
        for (int j = 0; j < frame_size.width; ++j) {
            fprintf(log, "%d ", mask[i*frame_size.width + j]);
        }
        fprintf(log, "\n");
    }
    fclose(log);
    */
    static CvMat* roiMask = cvCreateMat(frame_size.height,frame_size.width, CV_8UC1); 
    roiMask->data.ptr = mask;
    //mexPrintf("height: %d\n", frame_size.height);
    //mexPrintf("width: %d\n", frame_size.width);
    
    
    /*output data to matlab: the pixel features points mask*/
    //plhs[0] = mxCreateLogicalMatrix(1, frame_size.height*frame_size.width); //matlab initialize it to all 0s
	//mxLogical* res = (mxLogical*)mxGetData(plhs[0]);
    plhs[0] = mxCreateNumericMatrix(1, MAX_NUM_OF_FEATURES, mxUINT32_CLASS, mxREAL);
    int* resPrevX = (int*) mxGetData(plhs[0]);
    plhs[1] = mxCreateNumericMatrix(1, MAX_NUM_OF_FEATURES, mxUINT32_CLASS, mxREAL);
    int* resPrevY = (int*) mxGetData(plhs[1]);
    plhs[2] = mxCreateNumericMatrix(1, MAX_NUM_OF_FEATURES, mxUINT32_CLASS, mxREAL);
    int* resCurX = (int*) mxGetData(plhs[2]);
    plhs[3] = mxCreateNumericMatrix(1, MAX_NUM_OF_FEATURES, mxUINT32_CLASS, mxREAL);
    int* resCurY = (int*) mxGetData(plhs[3]);
	plhs[4] = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    int* featureCnt = (int*) mxGetData(plhs[4]);
    
    CvCapture *input_video = cvCaptureFromFile(fPath
		//"C:\\Users\\roman10\\Desktop\\cs5342-project\\6-code-in-matlab\\MovieData\\test2\\test2.avi"
		//"C:\\Users\\roman10\\Desktop\\cs5342-project\\6-code-in-matlab\\MovieData\\AC\\AC_Left_WithPhotomCorrection.avi"
		);
	if (input_video == NULL)
	{
		/* Either the video didn't exist OR it uses a codec OpenCV
		 * doesn't support.
		 */
		mexPrintf("Error: Can't open video %s.\n", fPath);
		return;
	}

	/* Create a windows called "Optical Flow" for visualizing the output.
	 * Have the window automatically change its size to match the output.
	 */
    //creates a window: int cvNamedWindow(const char* name, int flags);
    //name: name of the window, also used as the window identifier.
    //flags: currently only support CV_WINDOW_AUTOSIZE, if set, window size is automatically adjusted. 
	//cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);
    static IplImage *frame = NULL, *frame1 = NULL, *frame1_1C = NULL, *frame2_1C = NULL, *eig_image = NULL, *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;

    /* Go to the frame we want.  Important if multiple frames are queried in
     * the loop which they of course are for optical flow.  Note that the very
     * first call to this is actually not needed. (Because the correct position
     * is set outsite the for() loop.)
     */
    cvSetCaptureProperty( input_video, CV_CAP_PROP_POS_FRAMES, current_frame );

    /* Get the next frame of the video.
     * IMPORTANT!  cvQueryFrame() always returns a pointer to the _same_
     * memory location.  So successive calls:
     * frame1 = cvQueryFrame();
     * frame2 = cvQueryFrame();
     * frame3 = cvQueryFrame();
     * will result in (frame1 == frame2 && frame2 == frame3) being true.
     * The solution is to make a copy of the cvQueryFrame() output.
     */
    //cvQueryFrame: grabs and returns a frame from a camera or file
    frame = cvQueryFrame( input_video );
    if (frame == NULL)
    {
        /* Why did we get a NULL frame?  We shouldn't be at the end. */
        fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
        return;
    }
    /* Allocate another image if not already allocated.
     * Image has ONE channel of color (ie: monochrome) with 8-bit "color" depth.
     * This is the image format OpenCV algorithms actually operate on (mostly).
     */
    allocateOnDemand( &frame1_1C, frame_size, IPL_DEPTH_8U, 1 );
    /* Convert whatever the AVI image format is into OpenCV's preferred format.
     * AND flip the image vertically.  Flip is a shameless hack.  OpenCV reads
     * in AVIs upside-down by default.  (No comment :-))
     */
    //converts one image to another with an optional vertical flip
    //cvConvertImage(frame, frame1_1C, CV_CVTIMG_FLIP);
    cvConvertImage(frame, frame1_1C, 0);

    /* We'll make a full color backup of this frame so that we can draw on it.
     * (It's not the best idea to draw on the static memory space of cvQueryFrame().)
     */
    allocateOnDemand( &frame1, frame_size, IPL_DEPTH_8U, 3 );
    //cvConvertImage(frame, frame1, CV_CVTIMG_FLIP);
    cvConvertImage(frame, frame1, 0);

    /* Get the second frame of video.  Same principles as the first. */
    frame = cvQueryFrame( input_video );
    if (frame == NULL)
    {
        fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
        return;
    }
    allocateOnDemand( &frame2_1C, frame_size, IPL_DEPTH_8U, 1 );
    //cvConvertImage(frame, frame2_1C, CV_CVTIMG_FLIP);
    cvConvertImage(frame, frame2_1C, 0);

    /* Shi and Tomasi Feature Tracking! */

    /* Preparation: Allocate the necessary storage. */
    allocateOnDemand( &eig_image, frame_size, IPL_DEPTH_32F, 1 );
    allocateOnDemand( &temp_image, frame_size, IPL_DEPTH_32F, 1 );

    /* Preparation: This array will contain the features found in frame 1. */
    //2D point with floating-point coordinates
    CvPoint2D32f frame1_features[MAX_NUM_OF_FEATURES];

    /* Preparation: BEFORE the function call this variable is the array size
     * (or the maximum number of features to find).  AFTER the function call
     * this variable is the number of features actually found.
     */
    int number_of_features;

    /* I'm hardcoding this at 400.  But you should make this a #define so that you can
     * change the number of features you use for an accuracy/speed tradeoff analysis.
     */
    number_of_features = MAX_NUM_OF_FEATURES;

    /* Actually run the Shi and Tomasi algorithm!!
     * "frame1_1C" is the input image.
     * "eig_image" and "temp_image" are just workspace for the algorithm.
     * The first ".01" specifies the minimum quality of the features (based on the eigenvalues). 
     * The second ".01" specifies the minimum Euclidean distance between features.
     * "NULL" means use the entire input image.  You could point to a part of the image.
     * WHEN THE ALGORITHM RETURNS:
     * "frame1_features" will contain the feature points.
     * "number_of_features" will be set to a value <= 400 indicating the number of feature points found.
     */
    cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, &number_of_features, .001, .01, roiMask);

    /* Pyramidal Lucas Kanade Optical Flow! */

    /* This array will contain the locations of the points from frame 1 in frame 2. */
    CvPoint2D32f frame2_features[MAX_NUM_OF_FEATURES];

    /* The i-th element of this array will be non-zero if and only if the i-th feature of
     * frame 1 was found in frame 2.
     */
    char optical_flow_found_feature[MAX_NUM_OF_FEATURES];

    /* The i-th element of this array is the error in the optical flow for the i-th feature
     * of frame1 as found in frame 2.  If the i-th feature was not found (see the array above)
     * I think the i-th entry in this array is undefined.
     */
    float optical_flow_feature_error[MAX_NUM_OF_FEATURES];

    /* This is the window size to use to avoid the aperture problem (see slide "Optical Flow: Overview"). */
    CvSize optical_flow_window = cvSize(3,3);

    /* This termination criteria tells the algorithm to stop when it has either done 20 iterations or when
     * epsilon is better than .3.  You can play with these parameters for speed vs. accuracy but these values
     * work pretty well in many situations.
     */
    //CvTermCriteria: termination criteria for iterative algorithms: 20, 0.3
    CvTermCriteria optical_flow_termination_criteria
        = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );

    /* This is some workspace for the algorithm.
     * (The algorithm actually carves the image into pyramids of different resolutions.)
     */
    allocateOnDemand( &pyramid1, frame_size, IPL_DEPTH_8U, 1 );
    allocateOnDemand( &pyramid2, frame_size, IPL_DEPTH_8U, 1 );

    /* Actually run Pyramidal Lucas Kanade Optical Flow!!
     * "frame1_1C" is the first frame with the known features.
     * "frame2_1C" is the second frame where we want to find the first frame's features.
     * "pyramid1" and "pyramid2" are workspace for the algorithm.
     * "frame1_features" are the features from the first frame.
     * "frame2_features" is the (outputted) locations of those features in the second frame.
     * "number_of_features" is the number of features in the frame1_features array.
     * "optical_flow_window" is the size of the window to use to avoid the aperture problem.
     * "5" is the maximum number of pyramids to use.  0 would be just one level.
     * "optical_flow_found_feature" is as described above (non-zero iff feature found by the flow).
     * "optical_flow_feature_error" is as described above (error in the flow for this feature).
     * "optical_flow_termination_criteria" is as described above (how long the algorithm should look).
     * "0" means disable enhancements.  (For example, the second array isn't pre-initialized with guesses.)
     */
    cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features, optical_flow_window, 5, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria, 0 );

    /*copy the results to matlab interface output*/
    for (int i = 0; i < number_of_features; ++i) {
        if (optical_flow_found_feature[i]!=0) {
            //res[(int)frame1_features[i].y*frame_size.width + (int)frame1_features[i].x] = true;
            //mexPrintf("x: %d; y: %d\n", (int)frame1_features[i].x, (int)frame1_features[i].y);
            //res[(int)frame1_features[i].x*frame_size.height + (int)frame1_features[i].y] = true;
            resPrevX[i] = (int)frame1_features[i].x;
            resPrevY[i] = (int)frame1_features[i].y;
            resCurX[i] = (int)frame2_features[i].x;
            resCurY[i] = (int)frame2_features[i].y;
            //mexPrintf("%d:%d:%d:%d\n", resPrevX[i], resPrevY[i], resCurX[i], resCurY[i]);
            ++(*featureCnt);
        }
    }
    //mexPrintf("%d\n", *featureCnt);
    /* For debugging, let's draw the flow field. */
if (0) {
    int cnt = 0;
    for(int i = 0; i < number_of_features; i++)
    {
        /* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
        if ( optical_flow_found_feature[i] == 0 )	continue;
        ++cnt;
        int line_thickness;				line_thickness = 1;
        /* CV_RGB(red, green, blue) is the red, green, and blue components
         * of the color you want, each out of 255.
         */	
        CvScalar line_color;			line_color = CV_RGB(255,0,0);

        /* Let's make the flow field look nice with arrows. */

        /* The arrows will be a bit too short for a nice visualization because of the high framerate
         * (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
         */
        CvPoint p,q;
        p.x = (int) frame1_features[i].x;
        p.y = (int) frame1_features[i].y;
        q.x = (int) frame2_features[i].x;
        q.y = (int) frame2_features[i].y;
        //for debug:
        if ((p.x == 0) || (p.y == 0) || (q.x == 0) || (q.y == 0) ) {
            mexPrintf("pixel index starts at 0\n");
        }

        double angle;		angle = atan2( (double) p.y - q.y, (double) p.x - q.x );     //arc-tangent of (p.y-q.y)/(p.x-q.x)
        double hypotenuse;	hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );  

        /* Here we lengthen the arrow by a factor of three. */
        //q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
        //q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

        /* Now we draw the main line of the arrow. */
        /* "frame1" is the frame to draw on.
         * "p" is the point where the line begins.
         * "q" is the point where the line stops.
         * "CV_AA" means antialiased drawing.
         * "0" means no fractional bits in the center cooridinate or radius.
         */
        cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
        /* Now draw the tips of the arrow.  I do some scaling so that the
         * tips look proportional to the main line of the arrow.
         */			
        p.x = (int) (q.x + 9 * cos(angle + pi / 4));
        p.y = (int) (q.y + 9 * sin(angle + pi / 4));
        cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
        p.x = (int) (q.x + 9 * cos(angle - pi / 4));
        p.y = (int) (q.y + 9 * sin(angle - pi / 4));
        cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
    }
    /* Now display the image we drew on.  Recall that "Optical Flow" is the name of
     * the window we created above.
     */
    cvShowImage("Optical Flow", frame1);
    /* And wait for the user to press a key (so the user has time to look at the image).
     * If the argument is 0 then it waits forever otherwise it waits that number of milliseconds.
     * The return value is the key the user pressed.
     */
    int key_pressed;
    key_pressed = cvWaitKey(0);
}
    cvReleaseCapture(&input_video);
}
