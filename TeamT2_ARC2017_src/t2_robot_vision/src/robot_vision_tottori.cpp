/*
 * 鳥取大認識モジュール
 * 	キャリブレーション部→　：(9) CalibratedData.msg
 *	→認識統合部　　　　：(11) RecognizedItem.msg
 */

#include <robot_vision_tottori.h>

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/package.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

//#include <iostream>
//#include <fstream>
//#include <string>

//#include <stdlib.h>
//#include <float.h>
//#include <memory.h>
//#define _USE_MATH_DEFINES
//#include <math.h>

//using namespace std;

// Register nodelet
PLUGINLIB_EXPORT_CLASS(catch_robot_vision::robot_vision_tottori, nodelet::Nodelet);

namespace catch_robot_vision
{

robot_vision_tottori::robot_vision_tottori()
{
}

/*
 * Nodelet Destructor.
 */
robot_vision_tottori::~robot_vision_tottori()
{
}

/*
 * Initialize the nodelet.
 */
void robot_vision_tottori::onInit()
{
  ROS_INFO_STREAM( "robot_vision_tottori::onInit() start" );

  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();

  tottori_sub_ = nh.subscribe("calibrated_data", 10, &robot_vision_tottori::calibratedDataCallback, this);
  tottori_pub_ = nh.advertise<catch_robot_vision::RecognizedItem>("recognized_item", 10);


  // darknet初期化 はじめ by nishiyama
  ROS_INFO_STREAM("[Tottori DARKNET] Sart loading dictionary");

  char datacfg_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/mytrain.dataset";
  char names_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/mytrain.names.list";
  char names_opt[128] = "names";
  char cfgfile_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/yolo.cfg";
  //char weightfile_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/yolo_370000.weights";
  //char weightfile_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/yolo_480000.weights";
  //char weightfile_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/yolo_530000.weights";
  //char weightfile_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/yolo_680000.weights";
  //char weightfile_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/yolo_710000.weights";
  //char weightfile_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/yolo_720000.weights";
  //char weightfile_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/yolo_770000.weights";
  char weightfile_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/yolo_800000.weights";

  darknet_model.options = read_data_cfg(datacfg_filename);
  darknet_model.name_list = option_find_str(darknet_model.options, names_opt, names_filename);
  darknet_model.names = get_labels(darknet_model.name_list);
  darknet_model.net = parse_network_cfg(cfgfile_filename);
  load_weights(&darknet_model.net, weightfile_filename);
  set_batch_network(&darknet_model.net, 1);
  srand(2222222);

  ROS_INFO_STREAM("[Tottori DARKNET] End loading dictionary");
  // darknet初期化 おわり by nishiyama

  ROS_INFO_STREAM( "robot_vision_tottori::onInit() end" );
}


void robot_vision_tottori::calibratedDataCallback(const catch_robot_vision::CalibratedDataConstPtr &notice)
{
  int jn = notice->recog_target.job_no;
  ROS_INFO_STREAM("[START] job_no:" << jn << " robot_vision_tottori::calibratedDataCallback");
  catch_robot_vision::RecognizedItemPtr data(new catch_robot_vision::RecognizedItem);
  // 入力をスルーパス
  data->recog_target = notice->recog_target;
  data->calibrated_points.push_back(notice->calibrated_points[0]);

  //
  // 画像変換(sensor_msgs/Image -> cvMat[])
  //
  const catch_robot_vision::RecognitionTarget recog_target = notice->recog_target;
  std::vector<cv::Mat> cMat_v; // 実際には入力画像は１枚なので配列サイズ１
  for(std::vector<catch_robot_vision::CapturedData>::const_iterator itr = recog_target.data.begin();
      itr != recog_target.data.end();
      itr++){
    cv_bridge::CvImagePtr cv_color_ptr;
    try{
      if(itr->rgb.encoding == "rgb8"){
	cv_color_ptr = cv_bridge::toCvCopy(itr->rgb, sensor_msgs::image_encodings::RGB8);
      }else{
	ROS_ERROR_STREAM("catch_robot_vision:segment cam_id:" << itr->cam_id << " not compatible color encodings:" << itr->rgb.encoding);
	break;
      }
    }
    catch(cv_bridge::Exception& e){
      ROS_ERROR("catch_robot_vision:segment cam_id:%d, cv_bridge exception:%s", itr->cam_id,e.what());
      break;
    }
    //cv::cvtColor(cv_color_ptr->image,cv_color_ptr->image,CV_RGB2BGR);
    cMat_v.push_back(cv_color_ptr->image);

#if 0 // export input image
    {
      std::string src_root_path = ros::package::getPath("catch_robot_vision");
      std::string debug_path = src_root_path + "/data/debug/InputCheck_";
      std::stringstream ss;
      ss << debug_path << "jobNo_" << recog_target.job_no << "_camID_" << itr->cam_id;
      cv::imwrite(ss.str()+"_color.bmp", cv_color_ptr->image);
    }
#endif
  }


  //
  // ここで認識処理  by nishiyama
  //
  ROS_INFO_STREAM("[Tottori DARKNET] Start detection");
  //printf("Size %d %d %d\n", cMat_v[0].cols, cMat_v[0].rows, cMat_v[0].channels());
  //char debug_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/src/darknet/data/debug";
  char debug_filename[512] = "/home/dl-box/catkin_ws/src/catch_robot_vision/data/debug";
  char debug_current[1024];
  float hier_thresh = 0.5;
  //float thresh = 0.3;
  //float thresh = 0.2;
  float thresh = 0.5;
  float nms=.4;
  int i,j,k;

  // darknetの画像フォーマットに変換 by nishiyama
  image im = make_image(cMat_v[0].cols, cMat_v[0].rows, cMat_v[0].channels());
  for(i = 0; i < cMat_v[0].rows; ++i){
    for(k= 0; k < cMat_v[0].channels(); ++k){
      for(j = 0; j < cMat_v[0].cols; ++j){
        im.data[k*cMat_v[0].cols*cMat_v[0].rows + i*cMat_v[0].cols + j] = cMat_v[0].data[i*cMat_v[0].step + j*cMat_v[0].elemSize() + k]/255.;
      }
    }
  }
  //rgbgr_image(im);
  //save_image(im, debug_filename); // Debug

  // 学習済みモデルの画像サイズに変換 by nishiyama
  image sized = letterbox_image(im, darknet_model.net.w, darknet_model.net.h);
  //save_image(sized, debug_filename); // Debug
  layer l = darknet_model.net.layers[darknet_model.net.n-1];
  //printf("darknet_model.net.n-1 = %d\n", darknet_model.net.n-1);

  // 出力先の確保 by nishiyama
  box *boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
  float **probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
  for(j = 0; j < l.w*l.h*l.n; ++j){
    probs[j] = (float *)calloc(l.classes + 1, sizeof(float *));
  }

  // 学習済みモデルで検出 by nishiyama
  float *X = sized.data;
  network_predict(darknet_model.net, X);
  //get_region_boxes(l, im.w, im.h, darknet_model.net.w, darknet_model.net.h, thresh, probs, boxes, 0, 0, hier_thresh, 1); // old version
  get_region_boxes(l, im.w, im.h, darknet_model.net.w, darknet_model.net.h, thresh, probs, boxes, 0, 0, 0, hier_thresh, 1); // new version
  ROS_INFO_STREAM("[Tottori DARKNET] Start detection" << __LINE__);

  do_nms_obj(boxes, probs, l.w*l.h*l.n, l.classes, nms);
  ROS_INFO_STREAM("[Tottori DARKNET] Start detection" << __LINE__);

#define NISHIYAMA_PITCH_YAW
#ifdef NISHIYAMA_PITCH_YAW
  int32_t pitch_list[150] = {-90,90,-90,90,-90,90,0,0,0,0,-90,90,-90,90,-90,90,0,0,0,0,-90,90,0,0,0,0,-90,90,0,0,0,0,-90,90,0,0,0,0,-90,90,0,0,0,0,-90,90,-90,90,0,0,0,0,-90,90,0,0,0,0,-90,90,90,-90,90,-90,-90,90,-90,90,0,0,0,0,-90,90,0,0,0,0,-90,90,0,0,0,0,-90,90,-90,90,0,0,0,0,-90,90,-90,90,-90,0,-90,90,-90,0,0,0,0,90,-90,0,0,0,0,90,-90,90,-90,90,-90,90,-90,0,0,0,0,90,-90,0,0,0,0,90,-90,0,0,0,0,90,90,-90,90,-90,0,0,0,0,90,-90,90,-90,90,-90};
  int32_t yaw_list[150] ={0,0,0,0,0,0,0,90,180,270,0,0,0,0,0,0,0,90,180,270,0,0,0,90,180,270,0,0,0,90,180,270,0,0,0,90,180,270,0,0,0,90,180,270,0,0,0,0,0,90,180,270,0,0,0,90,180,270,0,0,0,0,0,0,0,0,0,0,0,90,180,270,0,0,0,90,180,270,0,0,0,90,180,270,0,0,0,0,0,90,180,270,0,0,0,0,0,0,0,0,0,0,90,180,270,0,0,0,90,180,270,0,0,0,0,0,0,0,0,0,90,180,270,0,0,0,90,180,270,0,0,0,90,180,270,0,0,0,0,0,0,90,180,270,0,0,0,0,0,0};
#endif

  // 座標を変換し結果のメッセージを送信 by nishiyama
  int *class_label = new int [l.w*l.h*l.n];
  printf("lsize = %d * %d * %d\n", l.w, l.h, l.n);
  for(i = 0; i < l.w*l.h*l.n; ++i){
    class_label[i] = max_index(probs[i], l.classes);
    //printf("probs[%d][%d] = %f\n",i, class_label[i], probs[i][class_label[i]]);
    if(probs[i][class_label[i]] > thresh){

      catch_robot_vision::SegmentedData seg_data;
      seg_data.module_index = 10; // 10:tottori_module
      seg_data.img_idx = 0; // 認識1回あたり入力画像1枚なので常に0
      seg_data.img_type = 1; // 0:depth, 1:color
      //seg_data.mask = *ros_image_ptr; // 鳥取大認識モジュールの場合は空でOK

      catch_robot_vision::ItemizedData item;
      item.module_index = 10; // 10:tottori_module

      box b = boxes[i];
      int left  = (b.x-b.w/2.)*im.w;
      int right = (b.x+b.w/2.)*im.w;
      int top   = (b.y-b.h/2.)*im.h;
      int bot   = (b.y+b.h/2.)*im.h;
      //int left  = (b.x-b.w/2.);
      //int right = (b.x+b.w/2.);
      //int top   = (b.y-b.h/2.);
      //int bot   = (b.y+b.h/2.);

      if(left < 0) left = 0;
      if(right > im.w-1) right = im.w-1;
      if(top < 0) top = 0;
      if(bot > im.h-1) bot = im.h-1;

      char sisei_array[1000][1000] = {};
      char *str_p;
      str_p = strtok(darknet_model.names[class_label[i]], "_");
      int j = 0;
      while(str_p != NULL){
        strcpy(sisei_array[j], str_p);
        str_p = strtok(NULL, "_");
        j++;
      }
      int class_id = atoi(sisei_array[0]);
      int roll = 0;
#ifdef NISHIYAMA_PITCH_YAW
      int32_t pitch = pitch_list[class_label[i]];
      int32_t yaw = yaw_list[class_label[i]];
#else
      int pitch = atoi(sisei_array[1]);
      int yaw = atoi(sisei_array[2]);
#endif
      
      // 検出結果の送信
      seg_data.sx = left;  // 検出結果
      seg_data.ex = right; // 検出結果
      seg_data.sy = top;  // 検出結果
      seg_data.ey = bot;  // 検出結果
      data->segmented_data.push_back(seg_data);

      item.category = class_id; // 認識結果
      item.yaw = yaw; // 角度推定結果
      item.pitch = pitch; // 角度推定結果
      item.roll = roll; // 角度推定結果
      item.posx = 0.0; // 位置推定結果、未実装
      item.posy = 0.0; // 位置推定結果、未実装
      item.posz = 0.0; // 位置推定結果、未実装
      item.score = probs[i][class_label[i]]*100; // 認識スコア
      item.seg_id = data->segmented_data.size()-1; // 対応するセグメント（何番目か）
      data->itemized_data.push_back(item);

      // Debug
      sprintf(debug_current, 
             "[Tottori DARKNET] ID:%d Score:%d Posi(sx ex sy ey):%d %d %d %d Roll:%d Pitch:%d Yaw:%d",
              item.category, 
              item.score, 
              seg_data.sx, seg_data.ex, seg_data.sy, seg_data.ey,
              item.roll, item.pitch, item.yaw);
      ROS_INFO_STREAM(debug_current);

    }
  }

  data->rcg_module_index = 10;
  tottori_pub_.publish(data);

  delete [] class_label;
  free(boxes);
  for(j = 0; j < l.w*l.h*l.n; ++j){
    free(probs[j]);
  }
  free(probs);

  free_image(im);
  free_image(sized);
   
  ROS_INFO_STREAM("[Tottori DARKNET] End detection");
 
// darknet検出 おわり  by nishiyama

end:
  ROS_INFO_STREAM("[END] job_no:" << jn << " robot_vision_tottori::calibratedDataCallback");
  return;
}

} // namespace catch_robot_vision

