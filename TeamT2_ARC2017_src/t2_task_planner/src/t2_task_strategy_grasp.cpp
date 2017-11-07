/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Toshiba Corporation, 
 *                     Toshiba Infrastructure Systems & Solutions Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Toshiba Corporation, nor the Toshiba
 *       Infrastructure Systems & Solutions Corporation, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#define T2_TASK_STRATEGY_GRASP_SRC
#include "t2_task_planner/t2_task_strategy_grasp.h"

/* システムヘッダ参照 */

/* 外部ヘッダ参照 */
#include "t2_task_planner/t2_task_strategy_component.h"

/* 内部定数定義 */
#define COMPILE_UPSIDE_PINCH                                    /* 上方挟持 */

/* 内部型定義 */
/* 内部変数定義 */

/* 内部関数定義 */
static void error_res(GRASP_OUTPUT_RSP_T *rspP, int32_t result);

/****************/
/* 外部関数処理 */
/****************/
//------------------------------
// 把持計画要求　応答作成部
//------------------------------
void graspCalc(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP)
{
  std::vector<Eigen::Affine3d> grasp_pos, w_grasp_pos, w_app_pos;
  Eigen::Vector3d gpx, gpy, gpz;                                /* GP姿勢の各軸成分 */
  Eigen::Vector3d r_gp, c_gp, c_shift;                          /* 座標間のベクトル */
  XmlRpc::XmlRpcValue matParam, infoParam;                      /* yaml読込用 */
  int mat, mat_num;
  bool ret;
  
  ros::NodeHandle nh;

  // std::cout << "release_location_list"<< reqP->release_location_list << std::endl;
  // std::cout << "place_id_list : " << reqP->place_id_list[0] << std::endl;
  
  for (;;)
  {
    /* 行列計算パラメータ展開 */
    ret = getParamTaskStrategy(&nh, &matParam);
    if (ret != true)
    {
      /* パラメータロードエラー */
      error_res(rspP, GRASP_RESULT_FAILED);
      break;                                                    /* 異常終了 */
    }
    
    /* ロボット原点(ワールド座標) */
    Eigen::Vector3d robot_origin(
      (double)matParam["r_pos"]["x"],
      (double)matParam["r_pos"]["y"],
      (double)matParam["r_pos"]["z"]
      );
    
    /* GP→AP変換行列 */
    Eigen::Affine3d gp2ap = Eigen::Affine3d::Identity();        /* (回転行列は)単位行列で初期化 */
    gp2ap.translation().x() = (double)matParam["offset_ap"]["x"]; /* 並進ベクトルを設定 */
    gp2ap.translation().y() = (double)matParam["offset_ap"]["y"]; /* 並進ベクトルを設定 */
    gp2ap.translation().z() = (double)matParam["offset_ap"]["z"]; /* 並進ベクトルを設定 */
    
    /* 把持リスト読み込み */
    ret = getParamGraspInfoList(reqP->cad_id, &nh, &infoParam);
    if (ret != true)
    {
      /* パラメータロードエラー */
      error_res(rspP, GRASP_RESULT_CAD_ID_ERR);
      break;                                                    /* 異常終了 */
    }
    
    /* レスポンス生成 */
    /* 結果 */
    rspP->result = GRASP_RESULT_SUCCESS;
    
    /* 総合スコア */
    rspP->total_score = (double)infoParam["score_exp"];
    
    /* 把持点数 */
    rspP->total_point = (uint32_t)((int)infoParam["gp_num_for_suction"]) + (uint32_t)((int)infoParam["gp_num_for_pinch"]);
    
    /* 把持点読み込み */
    rspP->grasp_point.resize(rspP->total_point);
    grasp_pos.resize(rspP->total_point);
    w_grasp_pos.resize(rspP->total_point);
    w_app_pos.resize(rspP->total_point);
    for (mat_num = 0; mat_num < (int)(rspP->total_point); mat_num++)
    {
      /* GP番号 */
      rspP->grasp_point[mat_num].gp_number = (uint32_t)((int)infoParam["gp"][mat_num]["gp_number"]);
      
      /* スコア */
      rspP->grasp_point[mat_num].score = (double)infoParam["gp"][mat_num]["score"];
      
      /* 把持方法 */
      rspP->grasp_point[mat_num].grasp_pattern = (std::string)infoParam["gp"][mat_num]["type"];
      
      /* 各種パラメータ */
      rspP->grasp_point[mat_num].length_of_pushing_for_suction = (double)infoParam["gp"][mat_num]["length_of_pushing_for_suction"];
      rspP->grasp_point[mat_num].threshold_of_vacuum_for_suction = (double)infoParam["gp"][mat_num]["threshold_of_vacuum_for_suction"];
      rspP->grasp_point[mat_num].suction_strength = (int32_t)((int)infoParam["gp"][mat_num]["suction_strength"]);
      rspP->grasp_point[mat_num].carry_speed = (double)infoParam["gp"][mat_num]["carry_speed"];
      rspP->grasp_point[mat_num].width_between_finger_for_pinch = (double)infoParam["gp"][mat_num]["width_between_finger_for_pinch"];
      rspP->grasp_point[mat_num].width_between_finger_for_release = (double)infoParam["gp"][mat_num]["width_between_finger_for_release"];
      rspP->grasp_point[mat_num].finger_intrusion_for_pinch = (double)infoParam["gp"][mat_num]["finger_intrusion_for_pinch"];
      rspP->grasp_point[mat_num].max_effort_for_pinch = (double)infoParam["gp"][mat_num]["max_effort_for_pinch"];
      
      /* 把持点抽出 */
      double m[16];                                             /* パラメータをダイレクトにmatrixに代入できなかったので、一旦ローカル配列に受ける */
      for (mat = 0; mat < 16; mat++)
      {
        m[mat] = (double)infoParam["gp"][mat_num]["mat4x4"][mat];
      }
      grasp_pos[mat_num].matrix() = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(&(m[0]));
    }

    /* アイテム姿勢のアラインメント */
    /* アイテム図心座標系のうちワールド座標系のz軸に近い軸を抽出 */
    Eigen::Vector3d::Index near_z_i;
    reqP->g_center_pos.linear().row(2).cwiseAbs().maxCoeff(&near_z_i);
    double dot_near_z = reqP->g_center_pos.linear().row(2)(near_z_i);

    /* z軸に近い軸とワールドz軸との角度を計算する */
    double angle_near_z = (dot_near_z > 0 ? 1 : -1) * acos(fabs(dot_near_z)) / M_PI * 180.0;

    if (fabs(angle_near_z) < (double)matParam["item_align_th_xy"])
    {
      /* z軸をアラインメントするための回転軸を計算し、アイテム座標系での表現に変換する */
      Eigen::Vector3d z_align_vec = reqP->g_center_pos.inverse().linear() * reqP->g_center_pos.linear().col(near_z_i).cross(Eigen::Vector3d::UnitZ());
      if (z_align_vec.norm() != 0) z_align_vec /= z_align_vec.norm();

      /* 回転軸まわりの回転行列を計算する */
      Eigen::Affine3d z_align = Eigen::Affine3d::Identity();
      z_align.linear() = Eigen::AngleAxisd(angle_near_z / 180.0 * M_PI, z_align_vec).toRotationMatrix();

      /* z軸に近い軸がワールドz軸と平行になるようにアイテム図心を回転する */
      reqP->g_center_pos = reqP->g_center_pos * z_align;

      /* アイテム図心座標系のうちワールド座標系のx,y軸に近い軸を抽出 */
      Eigen::Vector3d::Index near_x_i, near_y_i;
      reqP->g_center_pos.linear().row(0).cwiseAbs().maxCoeff(&near_x_i);
      reqP->g_center_pos.linear().row(1).cwiseAbs().maxCoeff(&near_y_i);

      /* x軸とワールドx軸との角度を計算する */
      // double angle_x = acos(reqP->g_center_pos.linear()(0,near_x_i)) / M_PI * 180.0;
      double angle_x = (reqP->g_center_pos.linear()(1,near_x_i) > 0 ? 1 : -1) * acos(reqP->g_center_pos.linear()(0,near_x_i)) / M_PI * 180.0;

      /* 90°の倍数に最も近くなる回転量を計算する */
      double mod_90 = -(fmod((angle_x+180.0+45.0), 90.0) - 45.0);

      if (fabs(mod_90) < (double)matParam["item_align_th_z"])
      {
        /* ワールドz軸まわりの回転行列を計算する */
        Eigen::Affine3d xy_align = Eigen::Affine3d::Identity();
        xy_align.linear() = Eigen::AngleAxisd(mod_90 / 180.0 * M_PI, reqP->g_center_pos.inverse().linear() * Eigen::Vector3d::UnitZ()).toRotationMatrix();

        /* x,y軸をアラインメントする */
        reqP->g_center_pos = reqP->g_center_pos * xy_align;
      }
    }
    
    /* 把持点をアイテム座標系からワールド座標系に変換 */
    for (mat_num = 0; mat_num < (int)(rspP->total_point); mat_num++)
    {
      w_grasp_pos[mat_num] = (reqP->g_center_pos) * grasp_pos[mat_num];
    }
    
    /* ΣwGPの姿勢行列をハンド進入方向に向ける(ロボットから見て、x:右方向, y:手前方向, z:下方向) */
    for (mat_num = 0; mat_num < (int)(rspP->total_point); mat_num++)
    {
      if (rspP->grasp_point[mat_num].grasp_pattern == "suction")
      {
        /* 吸着 */
        /* ロボット原点からGPまでのベクトル生成 */
        r_gp = w_grasp_pos[mat_num].translation() - robot_origin;
        
        /* GPからz成分だけのベクトルを抜き出す */
        gpz = w_grasp_pos[mat_num].linear().col(2);
        
        /* GPzとロボット向きベクトルの外積をとるとGPx */
        gpx = gpz.cross(r_gp);

        /* 上から見てr_gpの右側を向くようにGPxを修正する */
        if (r_gp.cross(gpx).z() > 0) gpx *= -1;
        
        /* GPxを長さで割って単位ベクトル化する */
        gpx /= gpx.norm();
        
        /* GPzとGPxの外積をとるとGPy */
        gpy = gpz.cross(gpx);
        
        /* GPyを長さで割って単位ベクトル化する */
        // gpy /= gpy.norm(); // 直行する単位化されたベクトル(GPz, GPx)同士の外積なので既に単位化されている
        
        /* 上記の計算をまとめてGPを更新する(ロボット方向に向けたGPになる) */
        w_grasp_pos[mat_num].linear().col(0) = gpx;
        w_grasp_pos[mat_num].linear().col(1) = gpy;
        // w_grasp_pos[mat_num].linear().col(2) = gpz; // 乗せ替えてるだけなので、やらなくても同じ
      }
      else
      {
        /* 挟持 */
        /*** ハンドが無理な姿勢になることを回避 ***/
        /* ロボット原点からGPまでのベクトル生成 */
        r_gp = w_grasp_pos[mat_num].translation() - robot_origin;
        r_gp.z() = 0.0;                                         /* 高さ成分は比較対象としないため、0で上書きする */
        
        /* GPからy成分だけのベクトルを抜き出す */
        gpy = w_grasp_pos[mat_num].linear().col(1);
        gpy.z() = 0.0;                                          /* 高さ成分は比較対象としないため、0で上書きする */
        
        /* 内積を判定し、y軸がロボット方向を向いていなかったら、x,y軸を180°回転させる */
        if (r_gp.dot(gpy) > 0)
        {
          /* 180°回転はパラメータ的な扱いでないため、行列はハードコーディングとする */
          Eigen::Affine3d xy180 = Eigen::Affine3d::Identity();  /* (回転行列は)単位行列で初期化 */
          xy180.linear() <<
            -1,  0,  0,
            0, -1,  0,
            0,  0,  1;
          
          w_grasp_pos[mat_num] = w_grasp_pos[mat_num] * xy180;
        }
        
#ifdef COMPILE_UPSIDE_PINCH
        /*** ロボット進入方向の変更 ***/
        /* 図心から把持点までのベクトルを算出 */
        c_gp = w_grasp_pos[mat_num].translation() - reqP->g_center_pos.translation();
        
        /* 算出したベクトルをスカラー倍する */
        /* (何倍でもいいが、ロボット進入方向からGPまでの進入方向成分を確保するのが目的) */
        c_shift = 2 * c_gp;
        
        /* ワールド座標に変換 */
        Eigen::Vector3d shift = reqP->g_center_pos.translation() + c_shift;
        
        /* ロボット進入原点を定義 */
        Eigen::Vector3d robot_shift_origin(0.0, 0.0, (double)matParam["r_pos"]["z"]);
        robot_shift_origin += shift;
        
        /* ロボット進入原点からGPまでのベクトル生成 */
        r_gp = w_grasp_pos[mat_num].translation() - robot_shift_origin;
        
        /*** 上方挟持のための把持点算出 ***/
        /* GPからy成分だけのベクトルを抜き出す */
        gpy = w_grasp_pos[mat_num].linear().col(1);
        
        /* GPyとロボット向きベクトルの外積をとるとGPx */
        gpx = gpy.cross(r_gp);
        
        /* GPxを長さで割って単位ベクトル化する */
        if (gpx.norm() != 0) 
        {
          gpx /= gpx.norm();
        
          /* GPxとGPyの外積をとるとGPz */
          gpz = gpx.cross(gpy);
        
          /* GPzを長さで割って単位ベクトル化する */
          // gpz /= gpz.norm(); // 直行する単位化されたベクトル(GPy, GPx)同士の外積なので既に単位化されている
        
          /* 上記の計算をまとめてGPを更新する(ロボット上方に向けたGPになる) */
          w_grasp_pos[mat_num].linear().col(0) = gpx;
          // w_grasp_pos[mat_num].linear().col(1) = gpy; // 乗せ替えてるだけなので、やらなくても同じ
          w_grasp_pos[mat_num].linear().col(2) = gpz;
        }
#endif /* COMPILE_UPSIDE_PINCH */
      }
    }
    
    /* 把持中継点(AP)の算出 */
    for (mat_num = 0; mat_num < (int)(rspP->total_point); mat_num++)
    {
      /* GPを法線方向に移動させた位置がAP */
      if (rspP->grasp_point[mat_num].grasp_pattern == "suction")
      {
        /* 吸着 */
        w_app_pos[mat_num] = w_grasp_pos[mat_num] * gp2ap;
      }
      else
      {
        /* 挟持 */ /* 挿入量を意識する必要あり */
        Eigen::Affine3d gp2ap_pinch = gp2ap;
        gp2ap_pinch.translation().z() -= (rspP->grasp_point[mat_num].finger_intrusion_for_pinch); /* 挿入量分オフセットする */
        
        w_app_pos[mat_num] = w_grasp_pos[mat_num] * gp2ap_pinch;
      }
    }
    
    for (mat_num = 0; mat_num < (int)(rspP->total_point); mat_num++)
    {
      rspP->grasp_point[mat_num].grasp_point_item = w_grasp_pos[mat_num];
      rspP->grasp_point[mat_num].approach_point_item = w_app_pos[mat_num];
    }
    
    /********/
    break;                                                      /* 正常終了 */
  }
}

//------------------------------
// 再スコアリング処理(Bin底判定)
//------------------------------
void reScoreBottom(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP)
{
  Eigen::Vector3d vecN(0, 0, 1);                                /* Bin底法線ベクトル */ /* ワールド座標Zと一致 */
  Eigen::Vector3d vecCalcTarget;                                /* 内積算出対象ベクトル(GPの算出軸成分) */
  double dotProduct;                                            /* 内積 */
  double scale;                                                 /* 倍率 */
  
  printRspDebug(rspP, "reScoreBottom before");
  
  for (std::size_t i=0; i<rspP->grasp_point.size(); i++)
  {
#ifdef COMPILE_UPSIDE_PINCH
    /* 内積計算対象を設定 */
    if (rspP->grasp_point[i].grasp_pattern == "suction")
    {
      /* 吸着 */
      vecCalcTarget = rspP->grasp_point[i].grasp_point_item.linear().col(2); /* GPのZ成分抽出 */
    }
    else
    {
      /* 挟持 */
      vecCalcTarget = reqP->g_center_pos.translation() - rspP->grasp_point[i].grasp_point_item.translation(); /* GPから図心までのベクトル(注：GPと図心は僅かでもズレていること) */
      vecCalcTarget /= vecCalcTarget.norm();
    }
#else /* COMPILE_UPSIDE_PINCH */
    vecCalcTarget = rspP->grasp_point[i].grasp_point_item.linear().col(2); /* GPのZ成分抽出 */
#endif /* COMPILE_UPSIDE_PINCH */
    
    /* 内積をとり、Bin底方向を除外する */
    dotProduct = vecN.dot(vecCalcTarget);                       /* 内積を算出 */
    if (dotProduct <= 0)
    {
      /* Bin上方(or真横)方向からのアプローチに再スコアリング */
      scale = (0.5 * fabs(dotProduct)) + 0.5;                   /* 倍率を算出 */
    }
    else
    {
      /* Bin底からのアプローチはしない */
      scale = 0.0;                                              /* 倍率を0に設定 */
    }
    rspP->grasp_point[i].score *= scale;                        /* スコアに倍率を掛け合わせる */
  }
  
  printRspDebug(rspP, "reScoreBottom after");
}

//------------------------------
// 再スコアリング処理(Y軸姿勢判定)
//------------------------------
void reScorePoseY(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP)
{
#ifdef COMPILE_UPSIDE_PINCH
  Eigen::Vector3d vecN(0, 0, 1);                                /* Bin底法線ベクトル */ /* ワールド座標Zと一致 */
  Eigen::Vector3d vecCalcTarget;                                /* 内積算出対象ベクトル(GPの算出軸成分) */
  double dotProduct;                                            /* 内積 */
  double theta;                                                 /* 角度 */
  double scale;                                                 /* 倍率 */
  
  printRspDebug(rspP, "reScorePoseY before");
  
  for (std::size_t i=0; i<rspP->grasp_point.size(); i++)
  {
    /* 挟持のみ対象 */
    if (rspP->grasp_point[i].grasp_pattern == "pinch")
    {
      vecCalcTarget = rspP->grasp_point[i].grasp_point_item.linear().col(1); /* GPのY成分抽出 */
      dotProduct = vecN.dot(vecCalcTarget);                     /* 内積をとり、cosθを算出 */
      /* 内積値を-1から1にクリップ */
      if (dotProduct < -1.0) dotProduct = -1.0;
      if (dotProduct >  1.0) dotProduct =  1.0;
      theta = std::acos(dotProduct);                            /* θを算出[rad] */
      scale = fabs(std::sin(theta));                            /* sinθの絶対値を倍率とする */
      rspP->grasp_point[i].score *= scale;                      /* スコアに倍率を掛け合わせる */
    }
  }
  
  printRspDebug(rspP, "reScorePoseY after");
#endif /* COMPILE_UPSIDE_PINCH */
}

//------------------------------
// 把持点再計算(GP個数の増減処理)
//------------------------------
void graspReCalc(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP)
{
  /* 把持点の数を増やす */

  /* すべてのgp_numberを100倍する */
  for (int mat_num = 0; mat_num < (int)(rspP->total_point); mat_num++)
  {
    rspP->grasp_point[mat_num].gp_number *= 100;
  }
  
  /* 行列計算パラメータ展開 */
  XmlRpc::XmlRpcValue matParam;                      /* yaml読込用 */
  bool ret;  
  ros::NodeHandle nh;
  ret = getParamTaskStrategy(&nh, &matParam);

  /* z軸方向に把持点を押しこむ */
  if ((bool)matParam["gp_offset_z_suction"] || (bool)matParam["gp_offset_z_pinch"])
  {
    /* GPをz軸方向に押しこむ */
    for (std::vector<GRASP_POINT_T>::iterator it = rspP->grasp_point.begin(); it != rspP->grasp_point.end(); it++)
    {
      if ((it->grasp_pattern == "suction" && (bool)matParam["gp_offset_z_suction"]) ||
          (it->grasp_pattern == "pinch" && (bool)matParam["gp_offset_z_pinch"])) 
      {
        double gp_offset;

        /* パラメータ読み込み */
        if (it->grasp_pattern == "suction")
        {
          gp_offset = (double)matParam["gpoz_offset_suction"];            /* (吸着)GP押し込み距離[m] */
        }
        else
        {
          gp_offset = (double)matParam["gpoz_offset_pinch"];              /* (挟持)GP押し込み距離[m] */
        }

        /* ワールド座標系におけるアイテム座標系のz軸ベクトル */
        Eigen::Vector3d axis_z = it->grasp_point_item.linear().col(2);

        it->grasp_point_item.translation() += gp_offset * axis_z;
      }
    }
  }
  /* z軸方向に把持点を増やす */
  else if ((bool)matParam["gp_increase_z"]) /* 押し込み機能ONの場合は働かない */
  {
    /* パラメータ読み込み */
    int best_score_num = (int)matParam["gpiz_best_score_num"];       /* 対象とするスコア上位のGP数 */
    int front_gp_num = (int)matParam["gpiz_front_gp_num"];           /* 奥側GP増加数 */
    int back_gp_num = (int)matParam["gpiz_back_gp_num"];             /* 手前側GP増加数 */
    double front_gp_step = (double)matParam["gpiz_front_gp_step"];   /* 奥側GP押し込み距離[m] */
    double back_gp_step = (double)matParam["gpiz_back_gp_step"];     /* 手前側GP押し込み距離[m] */
    double score_step = (double)matParam["gpiz_score_step"];         /* スコア調整刻み値 */

    /* スコア上位の把持点を抽出する */
    std::vector<double> score_list;
    std::vector<int> best_score_i;    
    if (best_score_num > (int)(rspP->total_point)) best_score_num = (int)(rspP->total_point);
    for (int mat_num = 0; mat_num < (int)(rspP->total_point); mat_num++)
    {
      score_list.push_back(rspP->grasp_point[mat_num].score);
    }
    for (unsigned int i = 0; i < best_score_num; i++)
    {
      std::vector<double>::iterator it = std::max_element(score_list.begin(), score_list.end());
      best_score_i.push_back(std::distance(score_list.begin(), it));
      *it = -DBL_MAX;
    }

    /* z軸方向オフセットリスト生成 */
    std::vector<double> offset_list;
    for (unsigned int i = 0; i < front_gp_num; i++) offset_list.push_back( front_gp_step*(double)(front_gp_num - i));
    for (unsigned int i = 0; i <  back_gp_num; i++) offset_list.push_back(- back_gp_step*(double)( back_gp_num - i));

    /* GPをz軸方向に増やす */
    std::vector<GRASP_POINT_T> new_grasp_point = rspP->grasp_point;
    GRASP_POINT_T new_gp_elem;
    unsigned int new_total_point = rspP->total_point;
    for (std::vector<int>::iterator it = best_score_i.begin(); it != best_score_i.end(); it++)
    {
      /* ワールド座標系におけるアイテム座標系のz軸ベクトル */
      Eigen::Vector3d axis_z = rspP->grasp_point[*it].grasp_point_item.linear().col(2);

      for (unsigned int offset_i = 0; offset_i < offset_list.size(); offset_i++)
      {
        new_gp_elem = rspP->grasp_point[*it];
        new_gp_elem.gp_number += 10*(offset_i + 1);
        new_gp_elem.score = rspP->grasp_point[*it].score - score_step * (offset_i < front_gp_num ? offset_i : offset_i+1);
        if (new_gp_elem.score < 0.0) new_gp_elem.score = 0.0;
        new_gp_elem.grasp_point_item.translation() += offset_list[offset_i] * axis_z;
        new_grasp_point.push_back(new_gp_elem);
        new_total_point++;
      }      

      /* 元のGPのスコアも調整する */
      new_grasp_point[*it].score -= score_step * front_gp_num;
    }

    rspP->grasp_point = new_grasp_point;
    rspP->total_point = new_total_point;
  }

  /* 1つの把持点に対し、z軸を中心に90°ずつ回転させた把持点を追加する */
  if ((bool)matParam["gp_increase_rot"])
  {
    std::vector<GRASP_POINT_T> new_grasp_point = rspP->grasp_point;
    GRASP_POINT_T new_gp_elem;
    unsigned int new_total_point = rspP->total_point;

    /* スコア上位の把持点を抽出する */
    int best_score_num = (int)matParam["gpir_best_score_num"];       /* 対象とするスコア上位のGP数 */
    std::vector<double> score_list;
    std::vector<int> best_score_i;    
    if (best_score_num > (int)(rspP->total_point)) best_score_num = (int)(rspP->total_point);
    for (int mat_num = 0; mat_num < (int)(rspP->total_point); mat_num++)
    {
      score_list.push_back(rspP->grasp_point[mat_num].score);
    }
    for (unsigned int i = 0; i < best_score_num; i++)
    {
      std::vector<double>::iterator it = std::max_element(score_list.begin(), score_list.end());
      best_score_i.push_back(std::distance(score_list.begin(), it));
      *it = -DBL_MAX;
    }
  
    /* Z軸を中心に90°, 180°, 270°回転させる回転行列を計算する */
    Eigen::Affine3d z90, z180, z270;
    z90 = z180 = z270 = Eigen::Affine3d::Identity();
    z90.linear() <<
      0, -1,  0,
      1,  0,  0,
      0,  0,  1;
    z180.linear() <<
      -1,  0,  0,
      0, -1,  0,
      0,  0,  1;
    z270.linear() <<
      0,  1,  0,
      -1,  0,  0,
      0,  0,  1;
  
    for (std::vector<int>::iterator it = best_score_i.begin(); it != best_score_i.end(); it++)
    {
      /* 吸着把持点の場合 */
      if (rspP->grasp_point[*it].grasp_pattern == "suction")
      {  
        /* 90°回転 */
        new_total_point++;
        new_gp_elem = rspP->grasp_point[*it];
        new_gp_elem.gp_number += 1;
        new_gp_elem.grasp_point_item = new_gp_elem.grasp_point_item * z90;
        new_gp_elem.approach_point_item = new_gp_elem.approach_point_item * z90;
        new_grasp_point.push_back(new_gp_elem);
  
        /* 180°回転 */
        new_total_point++;
        new_gp_elem = rspP->grasp_point[*it];
        new_gp_elem.gp_number += 2;
        new_gp_elem.grasp_point_item = new_gp_elem.grasp_point_item * z180;
        new_gp_elem.approach_point_item = new_gp_elem.approach_point_item * z180;
        new_grasp_point.push_back(new_gp_elem);
  
        /* 270°回転 */
        new_total_point++;
        new_gp_elem = rspP->grasp_point[*it];
        new_gp_elem.gp_number += 3;
        new_gp_elem.grasp_point_item = new_gp_elem.grasp_point_item * z270;
        new_gp_elem.approach_point_item = new_gp_elem.approach_point_item * z270;
        new_grasp_point.push_back(new_gp_elem);
      } 
      else if (rspP->grasp_point[*it].grasp_pattern == "pinch")
      {
        /* 180°回転 */
        new_total_point++;
        new_gp_elem = rspP->grasp_point[*it];
        new_gp_elem.gp_number += 2;
        new_gp_elem.grasp_point_item = new_gp_elem.grasp_point_item * z180;
        new_gp_elem.approach_point_item = new_gp_elem.approach_point_item * z180;
        new_grasp_point.push_back(new_gp_elem);
      }
    }
    rspP->grasp_point = new_grasp_point;
    rspP->total_point = new_total_point;
  }

  /* 把持点高さ制限(TOTE床吸引対策) */
  if ((bool)matParam["tote_gp_limit"]) {
    XmlRpc::XmlRpcValue container_info_array;                           /* yaml読込用 */
    ret = nh.getParam("/t2_database/container_info", container_info_array);
    Eigen::Vector3d tote_pos, tote_dim;
    double tote_gp_offset = (double)matParam["tote_gp_offset"];

    /* TOTEの位置とサイズを取得 */
    for (unsigned int i = 0; i < container_info_array.size(); ++i)
    {
      if (container_info_array[i]["name"] == "TOTE1") 
      {
        tote_pos.x() = container_info_array[i]["inside_position"][0];
        tote_pos.y() = container_info_array[i]["inside_position"][1];
        tote_pos.z() = container_info_array[i]["inside_position"][2];

        tote_dim.x() = container_info_array[i]["inside_dimensions"][0];
        tote_dim.y() = container_info_array[i]["inside_dimensions"][1];
        tote_dim.z() = container_info_array[i]["inside_dimensions"][2];
      }
    }

    /* アイテム図心のx,y座標がTOTE内か */
    if (reqP->g_center_pos.translation().x() > tote_pos.x()-tote_dim.x() && 
        reqP->g_center_pos.translation().x() < tote_pos.x()              &&
        reqP->g_center_pos.translation().y() > tote_pos.y()-tote_dim.y() &&
        reqP->g_center_pos.translation().y() < tote_pos.y())
    {
      /* GPの位置が床面+オフセットを下回らないように制限 */
      for (unsigned int i = 0; i < rspP->total_point; i++)
      {
        if (rspP->grasp_point[i].grasp_point_item.translation().z() < tote_pos.z() + tote_gp_offset) {
          rspP->grasp_point[i].grasp_point_item.translation().z() = tote_pos.z() + tote_gp_offset;
        }
      }    
    }
  }
}

/****************/
/* 内部関数処理 */
/****************/
//------------------------------
// 異常時レスポンス作成
//------------------------------
static void error_res(GRASP_OUTPUT_RSP_T *rspP, int32_t result)
{
  rspP->result = result;                                        /* 結果は上から渡してもらう */
  rspP->total_score = 0.0;                                      /* 総合スコア0 */
  rspP->total_point = 0;                                        /* 把持候補数0 */
  rspP->grasp_point.resize(rspP->total_point);                  /* 配列サイズ0 */
}

