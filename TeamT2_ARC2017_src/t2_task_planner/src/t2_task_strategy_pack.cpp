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

#define T2_TASK_STRATEGY_PACK_SRC
#include "t2_task_planner/t2_task_strategy_pack.h"

/* システムヘッダ参照 */

/* 外部ヘッダ参照 */
#include "t2_task_planner/t2_task_strategy_component.h"

/* 内部定数定義 */
/* 内部型定義 */
/* 内部変数定義 */

/* 内部関数定義 */

/****************/
/* 外部関数処理 */
/****************/
//------------------------------
// yaml決め打ち箱詰
//------------------------------
bool packCalcFix(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP, uint gp_idx, uint place_idx)
{
  bool ret = true;
  
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue strategyParam, fixRpParam;                /* yaml読込用 */
  Eigen::Affine3d rcf, rp, rap;
  
  if (ret == true)
  {
    /* 行列計算パラメータ展開 */
    ret = getParamTaskStrategy(&nh, &strategyParam);
  }
  
  if (ret == true)
  {
    /* ★決め打ちパラメータ展開 */
    ret = getParamFixRp(reqP->cad_id, &nh, &fixRpParam);
  }
  
  if (ret == true)
  {
    /* ★決め打ちplace_id比較 */
    if ((int)reqP->place_id_list[place_idx] != (int)fixRpParam["place_id"])
    {
      ret = false;
    }
  }
  
  if (ret == true)
  {
    /* 箱詰重心算出 */
    /* ★決め打ちmat4x4をロード */
    double m[16];                                               /* パラメータをダイレクトにmatrixに代入できなかったので、一旦ローカル配列に受ける */
    for (int i=0; i<16; i++)
    {
      m[i] = (double)fixRpParam["mat4x4"][i];
    }
    rcf.matrix() = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(&(m[0]));
  }
  
  if (ret == true)
  {
    /* GPからRPを算出(1次計算) */
    rp = rcf * reqP->g_center_pos.inverse() * rspP->grasp_point[gp_idx].grasp_point_item;
  }
  
  if (ret == true)
  {
    /* 回転対応 */
    Eigen::Vector3d vecN(0, 0, 1);                              /* ワールド座標Z */
    Eigen::Vector3d rpz = rp.linear().col(2);                   /* RPのz成分を抜き出す */
    Eigen::Affine3d cf2rotationCf = Eigen::Affine3d::Identity(); /* 図心を回転後図心にするための変換行列 */
    double dotProduct;                                          /* 内積 */
    double theta;                                               /* 角度 */
    
    /*** y軸で回す(x成分とz成分だけ残す) ***/
    Eigen::Vector3d rpz_xz(rpz.x(), 0, rpz.z());                /* RPzのy成分だけ0にしたベクトル*/
    if ((rpz.y() != 1.0) && (rpz.y() != -1.0))                  /* (SIM用)実機ではまず偽にならない */
    {
      rpz_xz /= rpz_xz.norm();                                  /* 単位化 */
      dotProduct = vecN.dot(rpz_xz);                            /* 内積をとり、cosθを算出 */
      theta = std::acos(dotProduct);                            /* θを算出[rad] */
      
      /* xz要素の向いてる向きを考慮 */
      theta = theta - M_PI;
      if (rpz.x() > 0)
      {
        theta = -theta;
      }
      
      /* y軸で回すための回転行列を生成 */
      cf2rotationCf.linear() <<
        std::cos(theta), 0, std::sin(theta),
        0, 1, 0,
        -std::sin(theta), 0, std::cos(theta);
      
      rcf.linear() = cf2rotationCf.linear() * rcf.linear();     /* 箱詰重心更新 */
    }
    
    /*** x軸で回す(y成分とz成分だけ残す) ***/
    Eigen::Vector3d rpz_yz(0, rpz.y(), rpz.z());                /* RPzのx成分だけ0にしたベクトル*/
    if ((rpz.x() != 1.0) && (rpz.x() != -1.0))                  /* (SIM用)実機ではまず偽にならない */
    {
      rpz_yz /= rpz_yz.norm();                                  /* 単位化 */
      dotProduct = vecN.dot(rpz_yz);                            /* 内積をとり、cosθを算出 */
      theta = std::acos(dotProduct);                            /* θを算出[rad] */
      
      /* yz要素の向いてる向きを考慮 */
      theta = theta - M_PI;
      if (rpz.y() < 0)
      {
        theta = -theta;
      }
      
      /* x軸で回すための回転行列を生成 */
      cf2rotationCf.linear() <<
        1, 0, 0,
        0, std::cos(theta), -std::sin(theta),
        0, std::sin(theta), std::cos(theta);
      
      rcf.linear() = cf2rotationCf.linear() * rcf.linear();     /* 箱詰重心更新 */
    }
    
    /*** z要素に対する回転(x軸で回す) ***/
    if (rpz.z() == 1.0)                                         /* (SIM用)実機ではまず真にならない */
    {
      theta = M_PI;
      
      /* x軸で回すための回転行列を生成 */
      cf2rotationCf.linear() <<
        1, 0, 0,
        0, std::cos(theta), -std::sin(theta),
        0, std::sin(theta), std::cos(theta);
      
      rcf.linear() = cf2rotationCf.linear() * rcf.linear();     /* 箱詰重心更新 */
    }
  }
  
  if (ret == true)
  {
    /* RP再計算(2次計算) */
    rp = rcf * reqP->g_center_pos.inverse() * rspP->grasp_point[gp_idx].grasp_point_item;
    
    /* 実際にリリース(真空破壊)する場所は、算出したRPより少し上 */
    if (rspP->grasp_point[gp_idx].grasp_pattern == "suction")
    {
      rp.translation().z() += (double)strategyParam["offset_rp_suction"];
    }
    else
    {
      rp.translation().z() += (double)strategyParam["offset_rp_pinch"];
    }
  }
  
  if (ret == true)
  {
    /* RPからRAPを算出 */
    /* RAPはダンボール上面からアイテム高さ分上空になるはず */
    rap = rp;
    rap.translation().z() += (double)strategyParam["offset_rap"];
  }
  
  if (ret == true)
  {
    /* 結果格納 */
    rspP->grasp_point[gp_idx].rp = rp;
    rspP->grasp_point[gp_idx].rap = rap;
  }
  
  return ret;
}

//------------------------------
// Voxel箱詰
//------------------------------
bool packCalc(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP, uint gp_idx, uint place_idx, bool protrude_flg)
{
  bool ret = true;
  
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue infoParam, matParam;                      /* yaml読込用 */
  uint itemIdx = UINT_MAX;
  
  double item_row, item_col, item_high;                         /* itemの辺の長さ */
  double theta = 0;                                             /* ナナメ箱詰め[rad] */
  Eigen::Vector3d vecStore;                                     /* RP図心の座標 */
  bool xy_rot = false;                                          /* xy回転をしたかどうかのフラグ */
  
  Eigen::Affine3d rcf;                                          /* RP図心の姿勢と座標 */
  Eigen::Affine3d rp, rap;
  
  int book_flag;                                                /* Bookタイプのアイテム把持フラグ */
  
  if (ret == true)
  {
    /* 行列計算パラメータ展開 */
    ret = getParamTaskStrategy(&nh, &matParam);
  }
  
  if (ret == true)
  {
    /* 把持リスト展開 */
    ret = getParamGraspInfoList(reqP->cad_id, &nh, &infoParam);
    if (ret == true)
    {
      /* 該当gp_numberが、grasp_info_listのitemのどのindexか検索する */
      itemIdx = findGraspInfoIdx(&infoParam, (uint32_t)(rspP->grasp_point[gp_idx].gp_number / 100));  /* オリジナルのgp_numberと比較 */
      if (itemIdx == UINT_MAX)
      {
        ROS_ERROR("ParameterServer(grasp_info_list) can't find gp_number."); /* gp_numberが見つからない */
        ret =  false;
      }
      else
      {
        /* 把持点に対するxyzの長さを取得 */
        item_row = (double)infoParam["gp"][itemIdx]["y_length_of_item_in_gp_axis"];
        item_col = (double)infoParam["gp"][itemIdx]["x_length_of_item_in_gp_axis"];
        item_high = (double)infoParam["gp"][itemIdx]["z_length_of_item_in_gp_axis"];
        
        /* アイテム膨らませ対応 */
        if (item_row <= (double)matParam["item_plump_th"])
        {
          item_row += (double)matParam["item_plump_len_under"];
        }
        else
        {
          item_row += (double)matParam["item_plump_len_over"];
        }
        
        if (item_col <= (double)matParam["item_plump_th"])
        {
          item_col += (double)matParam["item_plump_len_under"];
        }
        else
        {
          item_col += (double)matParam["item_plump_len_over"];
        }
        
        /* アイテムがBookタイプで、かつ把持点が本の表表紙または裏表紙にあるか判断 */
        book_flag = (int)infoParam["book_flag"];
        if (book_flag != 0)
        {
          Eigen::Vector3d gpz = rspP->grasp_point[gp_idx].grasp_point_item.linear().col(2);
          if (fabs(gpz.z()) < fabs(gpz.x()) || fabs(gpz.z()) < fabs(gpz.y())) book_flag = 0;
        }
      }
    }
  }
  
  if (ret == true)
  {
    /* itemが収まる場所を検索(収まる場所のワールド座標を算出) */
    ret = findRp(reqP->place_id_list[place_idx], item_row, item_col, item_high, protrude_flg, &vecStore);
    if (ret == false)
    {
      /* 縦横90°回転リトライ */
      ret = findRp(reqP->place_id_list[place_idx], item_col, item_row, item_high, protrude_flg, &vecStore);
      if (ret == true)
      {
        /* 回転箱詰め成功リトライ */
        xy_rot = true;
      }
    }
    if (ret == false)
    {
      /* ナナメ箱詰め */
      double placeRow, placeCol, placeHigh;
      ret = getPlaceLength(reqP->place_id_list[place_idx], &placeRow, &placeCol, &placeHigh);
      if (ret == true)
      {
        theta = std::atan(placeRow / placeCol);                 /* θを算出[rad] */
        
        double itemColNew = item_col * std::cos(theta) + item_row * std::sin(theta);
        double itemRowNew = item_col * std::sin(theta) + item_row * std::cos(theta);
        item_col = itemColNew;
        item_row = itemRowNew;
        
        ret = findRp(reqP->place_id_list[place_idx], item_row, item_col, item_high, protrude_flg, &vecStore);
      }
    }
  }
  
  if (ret == true)
  {
    /**** RP図心算出 ****/
    
    /*** 箱詰目標となるRP姿勢を選択(カメラ位置考慮) ***/
    Eigen::Affine3d preRp = selectPreRp(reqP->place_id_list[place_idx], xy_rot);
    
    /*** 把持情報リストのGPをロード ***/
    Eigen::Affine3d gp;
    double m[16];                                               /* パラメータをダイレクトにmatrixに代入できないので、一旦ローカル配列に受ける */
    for (int i=0; i<16; i++)
    {
      m[i] = (double)infoParam["gp"][itemIdx]["mat4x4"][i];
    }
    gp.matrix() = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> >(&(m[0]));
    /* 並進ベクトル不要のため0埋め */
    gp.translation().x() = 0.0;
    gp.translation().y() = 0.0;
    gp.translation().z() = 0.0;
    
    /*** リリース点の図心算出 ***/
    rcf.linear() = preRp.linear() * gp.linear().inverse();
    rcf.translation().x() = vecStore.x();
    rcf.translation().y() = vecStore.y();
    rcf.translation().z() = vecStore.z();
    
    /*** 図心ナナメ対応 ***/
    Eigen::Affine3d naname = Eigen::Affine3d::Identity();
    naname.linear() <<
      std::cos(theta), std::sin(theta), 0,
      -std::sin(theta), std::cos(theta), 0,
      0, 0, 1;
    rcf.linear() = naname.linear() * rcf.linear();
  }
  
  if (ret == true)
  {
    /* はみ出し量算出 */
    rspP->grasp_point[gp_idx].protrude_length = calcProtrude(reqP->place_id_list[place_idx], rcf.translation(), item_high);
  }
  
  if (ret == true)
  {
    /* RP算出 */
    rp = rcf * reqP->g_center_pos.inverse() * rspP->grasp_point[gp_idx].grasp_point_item;
    
    if (book_flag != 0) /* Bookタイプアイテムの場合 */
    {
      /* 本の幅と厚みを取得 */
      /* 現在のところ本の幅はx方向長さ、厚みはz方向長さであることが前提 */
      double width = (double)infoParam["x_length_of_item"];
      double depth = (double)infoParam["z_length_of_item"];
      
      /* 設置位置底面からRPまでの高さ(本の幅-オフセット暫定5mm)を計算 */
      double floor_rp = width - 0.005;
      
      /* 現在RPを箱の中に設定できないため、下限を設けている(暫定150mm) */
      floor_rp = (floor_rp < 0.150 ? 0.150 : floor_rp);
     
      /* RPの高さを再設定 */
      rp.translation().z() = rcf.translation().z() - depth/2 + floor_rp;
    }
    else /* 通常アイテムの場合 */
    {
      /* 実際にリリース(真空破壊)する場所は、算出したRPより少し上 */
      if (rspP->grasp_point[gp_idx].grasp_pattern == "suction")
      {
        rp.translation().z() += (double)matParam["offset_rp_suction"];
      }
      else
      {
        rp.translation().z() += (double)matParam["offset_rp_pinch"];
      }
    }
  }
  
  if (ret == true)
  {
    /* RP姿勢補正 */
    /*** ハンドが無理な姿勢になることを回避 ***/
    /* ロボット原点(ワールド座標) */
    Eigen::Vector3d robot_origin(
      (double)matParam["r_pos"]["x"],
      (double)matParam["r_pos"]["y"],
      (double)matParam["r_pos"]["z"]
    );
    /* ロボット原点からRPまでのベクトル生成 */
    Eigen::Vector3d r_rp = rp.translation() - robot_origin;
    r_rp.z() = 0.0;                                           /* 高さ成分は比較対象としないため、0で上書きする */
    
    /* RPからy成分だけのベクトルを抜き出す */
    Eigen::Vector3d rpy = rp.linear().col(1);
    rpy.z() = 0.0;                                            /* 高さ成分は比較対象としないため、0で上書きする */
    
    /* 内積を判定し、y軸がロボット方向を向いていなかったら、x,y軸を180°回転させる */
    if (r_rp.dot(rpy) > 0)
    {
      /* リリース時のアイテム図心を計算する */
      Eigen::Affine3d r_center_pos = rp * rspP->grasp_point[gp_idx].grasp_point_item.inverse() * reqP->g_center_pos;
      
      /* RPからz成分だけのベクトルを抜き出し、アイテム座標系での表現に変換する */
      Eigen::Vector3d rpz = r_center_pos.inverse().linear() * rp.linear().col(2);
      
      /* rpzを軸として180°回転させる回転行列を計算する */
      Eigen::Affine3d xy180 = Eigen::Affine3d::Identity();
      xy180.linear() = Eigen::AngleAxisd(M_PI, rpz).toRotationMatrix();
      
      /* rpzを軸としてアイテム図心を180°回転する */
      r_center_pos = r_center_pos * xy180;
      
      /* 回転後のアイテム図心に合わせてRPを再計算する */
      rp = r_center_pos * reqP->g_center_pos.inverse() * rspP->grasp_point[gp_idx].grasp_point_item;
    }

    /* 挟持の場合のみ上方リリース */
    if (rspP->grasp_point[gp_idx].grasp_pattern == "pinch")
    {
      /* RPからz成分だけのベクトルを抜き出す */
      Eigen::Vector3d rpz = rp.linear().col(2);      

      /* rpzをz軸下向きに合わせるための回転軸を計算し、RP座標系での表現に変換する */
      Eigen::Vector3d z_rot_axis_w;
      if (reqP->place_id_list[place_idx] == 15)  /* リリース先がBOX3の場合、姿勢を傾ける */
      {
        z_rot_axis_w <<  0.1, 
                        -0.1,
                        -1.0;
        z_rot_axis_w /= z_rot_axis_w.norm();
        std::cout << "z_rot_axis_w : " << z_rot_axis_w << std::endl;      
      }
      else
      {
        z_rot_axis_w = -Eigen::Vector3d::UnitZ();
      }
      Eigen::Vector3d z_rot_axis = rp.inverse().linear() * rpz.cross(z_rot_axis_w);
      double z_dot = rpz.dot(z_rot_axis_w);
      double z_rot_angle = acos(z_dot);

      if (z_rot_axis.norm() > 0.1) /* <- 0.0 */
      {
        /* 回転軸ベクトルを正規化 */
        z_rot_axis /= z_rot_axis.norm();
        
        /* 回転軸まわりの回転行列を計算する */
        Eigen::Affine3d z_rot = Eigen::Affine3d::Identity();
        z_rot.linear() = Eigen::AngleAxisd(z_rot_angle, z_rot_axis).toRotationMatrix();

        /* rpのz軸をワールドz軸下向きに合わせる */
        rp = rp * z_rot;
      }
    }
  }
  
  if (ret == true)
  {
    /* RPからRAPを算出 */
    /* RAPはダンボール上面からアイテム高さ分上空になるはず */
    
    if (book_flag != 0) /* Bookタイプアイテムの場合 */
    {
      /* 本の幅を取得 */
      /* 現在のところ本の幅はx方向長さであることが前提 */
      double width = (double)infoParam["x_length_of_item"];
      
      /* 現在のRPからリリース時のアイテム図心を計算 */
      Eigen::Affine3d r_center_pos = rp * rspP->grasp_point[gp_idx].grasp_point_item.inverse() * reqP->g_center_pos;
      
      /* アイテム姿勢からx成分だけのベクトルを抜き出す */
      Eigen::Vector3d rcx = r_center_pos.linear().col(0);
      
      /* RPからアイテム図心へ向かうベクトルを計算 */
      Eigen::Vector3d rp_rc = r_center_pos.translation() - rp.translation();
      
      /* RPから本の開く側へのx方向変位を計算 */
      double rp_edge = rcx.dot(rp_rc) + width/2 * (book_flag == 1 ? 1 : -1);
      
      /* 本の開く側までRAPを移動 */
      rap = rp;
      rap.translation() += rcx * rp_edge;
      rap.translation().z() += (double)matParam["offset_rap"];
    }
    else /* 通常アイテムの場合 */
    {
      rap = rp;
      rap.translation().z() += (double)matParam["offset_rap"];
    }
  }
  
  if (ret == true)
  {
    /* 結果格納 */
    rspP->grasp_point[gp_idx].rp = rp;
    rspP->grasp_point[gp_idx].rap = rap;
  }
  
  return ret;
}

//------------------------------
// place上方をRPとする
// 
// (注)上方と言うものの底面ごとに空率を計算し、閾値(例：90%)を超えた面をRPとする。
//------------------------------
bool packPlaceUpside(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP, uint gp_idx, uint place_idx)
{
  bool ret = true;
  
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue strategyParam, infoParam;
  
  uint itemIdx;
  double item_high = 0.0;
  
  if (ret == true)
  {
    /* 行列計算パラメータ展開 */
    ret = getParamTaskStrategy(&nh, &strategyParam);
  }
  
  if (ret == true)
  {
    /* 把持リスト展開 */
    ret = getParamGraspInfoList(reqP->cad_id, &nh, &infoParam);
    if (ret == true)
    {
      /* 該当gp_numberが、grasp_info_listのitemのどのindexか検索する */
      itemIdx = findGraspInfoIdx(&infoParam, (uint32_t)(rspP->grasp_point[gp_idx].gp_number / 100));  /* オリジナルのgp_numberと比較 */
      if (itemIdx == UINT_MAX)
      {
        ROS_ERROR("ParameterServer(grasp_info_list) can't find gp_number."); /* gp_numberが見つからない */
        ret =  false;
      }
    }
  }
  
  if (ret == true)
  {
    /* place上方姿勢を取得し、RPとする */
    ret = getPlaceUpsidePose(reqP->place_id_list[place_idx], &(rspP->grasp_point[gp_idx].rp));
  }
  
  if (ret == true)
  {
    /* 底面検索 */
    /* アイテム3辺の最大値 */
    double item_len_max = (double)infoParam["gp"][itemIdx]["x_length_of_item_in_gp_axis"];
    if (item_len_max < (double)infoParam["gp"][itemIdx]["y_length_of_item_in_gp_axis"])
    {
      item_len_max = (double)infoParam["gp"][itemIdx]["y_length_of_item_in_gp_axis"];
    }
    if (item_len_max < (double)infoParam["gp"][itemIdx]["z_length_of_item_in_gp_axis"])
    {
      item_len_max = (double)infoParam["gp"][itemIdx]["z_length_of_item_in_gp_axis"];
    }
    
    /* 格納先row/colの最小値 */
    double placeRow, placeCol, placeHigh;
    ret = getPlaceLength(reqP->place_id_list[place_idx], &placeRow, &placeCol, &placeHigh);
    if (ret == true)
    {
      double place_len_min = placeRow;
      if (place_len_min > placeCol)
      {
        place_len_min = placeCol;
      }
      
      if ((item_len_max * 2) < place_len_min)
      {
        /* 空率が閾値(例：90%)を超えた面の高さをRPzに更新する */
        ret = findFreeLayerCoordZ(reqP->place_id_list[place_idx], &(rspP->grasp_point[gp_idx].rp.translation().z()));
      }
    }
  }
  
  if (ret == true)
  {
    if (reqP->single_gp == (uint)0)
    {
      /* 通常 */
      item_high = (double)infoParam["gp"][itemIdx]["z_length_of_item_in_gp_axis"];
    }
    else
    {
      /* 平面検出 */
      /* yamlのスコアが0より大きい把持点の中で、最大アイテム高さを検索 */
      for (std::size_t i=0; i<infoParam["gp"].size(); i++)
      {
        if ((double)infoParam["gp"][i]["score"] > 0.0)
        {
          if (item_high < (double)infoParam["gp"][i]["z_length_of_item_in_gp_axis"])
          {
            item_high = (double)infoParam["gp"][i]["z_length_of_item_in_gp_axis"];
          }
        }
      }
    }
    
    /* RP更新 */
    rspP->grasp_point[gp_idx].rp.translation().z() += item_high;
  }
  
  if (ret == true)
  {
    /* はみ出し量算出 */
    rspP->grasp_point[gp_idx].protrude_length = calcProtrudeCoord(reqP->place_id_list[place_idx], rspP->grasp_point[gp_idx].rp.translation().z());
  }

  if (ret == true)
  {
    /* 挟持の場合のみ上方リリース */
    if (rspP->grasp_point[gp_idx].grasp_pattern == "pinch")
    {
      /* RPからz成分だけのベクトルを抜き出す */
      Eigen::Vector3d rpz = rspP->grasp_point[gp_idx].rp.linear().col(2);      

      /* rpzをz軸下向きに合わせるための回転軸を計算し、RP座標系での表現に変換する */
      Eigen::Vector3d z_rot_axis_w;
      if (reqP->place_id_list[place_idx] == 15)  /* リリース先がBOX3の場合、姿勢を傾ける */
      {
        z_rot_axis_w <<  0.1, 
                        -0.1,
                        -1.0;
        z_rot_axis_w /= z_rot_axis_w.norm();
        std::cout << "z_rot_axis_w : " << z_rot_axis_w << std::endl;      
      }
      else
      {
        z_rot_axis_w = -Eigen::Vector3d::UnitZ();
      }
      Eigen::Vector3d z_rot_axis = rspP->grasp_point[gp_idx].rp.inverse().linear() * rpz.cross(z_rot_axis_w);
      double z_dot = rpz.dot(z_rot_axis_w);
      double z_rot_angle = acos(z_dot);

      if (z_rot_axis.norm() > 0.1) /* <- 0.0 */
      {
        /* 回転軸ベクトルを正規化 */
        z_rot_axis /= z_rot_axis.norm();
        
        /* 回転軸まわりの回転行列を計算する */
        Eigen::Affine3d z_rot = Eigen::Affine3d::Identity();
        z_rot.linear() = Eigen::AngleAxisd(z_rot_angle, z_rot_axis).toRotationMatrix();

        /* rpのz軸をワールドz軸下向きに合わせる */
        rspP->grasp_point[gp_idx].rp = rspP->grasp_point[gp_idx].rp * z_rot;
      }
    }
  }

  
  if (ret == true)
  {
//    /* 実際にリリース(真空破壊)する場所は、算出したRPより少し上 */
//    if (rspP->grasp_point[gp_idx].grasp_pattern == "suction")
//    {
//      rspP->grasp_point[gp_idx].rp.translation().z() += (double)strategyParam["offset_rp_suction"];
//    }
//    else
//    {
//      rspP->grasp_point[gp_idx].rp.translation().z() += (double)strategyParam["offset_rp_pinch"];
//    }
    
    /* RAP算出 */
    rspP->grasp_point[gp_idx].rap = rspP->grasp_point[gp_idx].rp;
    rspP->grasp_point[gp_idx].rap.translation().z() += (double)strategyParam["offset_rap"];
  }
  
  if (ret == true)
  {
    /* スコア上書き */
    rspP->grasp_point[gp_idx].score = (double)strategyParam["place_upside_rp_score"];
  }
  else
  {
    /* 万が一、エラーが起きたときは0に上書きしておく */
    rspP->grasp_point[gp_idx].score = 0.0;
  }
  
  return ret;
}

//------------------------------
// Amnesty上方をRPとする
//------------------------------
bool packAmnestyUpside(GRASP_INPUT_REQ_T *reqP, GRASP_OUTPUT_RSP_T *rspP, uint gp_idx, uint place_idx)
{
  bool ret = true;
  
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue strategyParam;
  
  if (ret == true)
  {
    /* 行列計算パラメータ展開 */
    ret = getParamTaskStrategy(&nh, &strategyParam);
  }
  
  if (ret == true)
  {
    /* place上方姿勢を取得し、RPとする */
    ret = getPlaceUpsidePose(reqP->place_id_list[place_idx], &(rspP->grasp_point[gp_idx].rp));
  }
  
  if (ret == true)
  {
    /* Amnesty用RP算出 */
    rspP->grasp_point[gp_idx].rp.translation().z() += (double)strategyParam["offset_rp_amnesty"];
    
    /* RAP算出 */
    rspP->grasp_point[gp_idx].rap = rspP->grasp_point[gp_idx].rp;
    rspP->grasp_point[gp_idx].rap.translation().z() += (double)strategyParam["offset_rap"];
    
    /* はみ出し量算出 */
    rspP->grasp_point[gp_idx].protrude_length = calcProtrudeCoord(reqP->place_id_list[place_idx], rspP->grasp_point[gp_idx].rp.translation().z());
  }
  
  if (ret == true)
  {
    /* スコア上書き */
    rspP->grasp_point[gp_idx].score = (double)strategyParam["place_upside_rp_score"];
  }
  else
  {
    /* 万が一、エラーが起きたときは0に上書きしておく */
    rspP->grasp_point[gp_idx].score = 0.0;
  }
  
  return ret;
}

/****************/
/* 内部関数処理 */
/****************/

