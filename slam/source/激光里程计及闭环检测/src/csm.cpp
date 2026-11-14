//
// Created by ycai on 2020/11/4.
//
#include "csm.h"
#include <iostream>

//#define _DEBUG


CSM::CSM()
{
    /*constructor*/
    egsl_ = new egsl();
}

CSM::~CSM(){
    /*de-constructor*/
    if (egsl_)
    delete egsl_;

}


void CSM::sm_icp(struct sm_params*params, struct sm_result*res, double* num_corres_ratio)
{
    res->valid = 0;

    LDP laser_ref  = params->laser_ref;
    LDP laser_sens = params->laser_sens;

    egsl_->egsl_push_named("sm_icp");


    if(params->use_corr_tricks || params->debug_verify_tricks)
        ld_create_jump_tables(laser_ref);//预先求出跳跃表，存储在laser_ref.up_bigger(up_smaller,down_bigger,down_smaller)

    ld_compute_cartesian(laser_ref);//转换到激光坐标系下的笛卡尔坐标-laser_ref
    ld_compute_cartesian(laser_sens);


    gsl_vector * x_new = gsl_vector_alloc(3);//预分配存储空间，用于存储算出来的位姿偏差
    gsl_vector * x_old = egsl_->vector_from_array(3, params->first_guess);

    double error;
    int iterations;
    int nvalid;

    if(!icp_loop(params, x_old->data, x_new->data, &error, &nvalid, &iterations, num_corres_ratio))
    {
        printf("icp: ICP failed for some reason. \n");
        res->valid = 0;
        res->iterations = iterations;
        res->nvalid = 0;

    }
    else
    {
        /* It was succesfull */
        double best_error = error;
        gsl_vector * best_x = gsl_vector_alloc(3);
        gsl_vector_memcpy(best_x, x_new);

        /* At last, we did it. */
        res->valid = 1;
        egsl_->vector_to_array(best_x, res->x);

#ifdef _DEBUG
        printf("icp: final x =  %s  \n", egsl_->gsl_friendly_pose(best_x));
#endif
        if(params->do_compute_covariance)  //目前存在算法模型和计算时长的问题，以及localization和帧间匹配的模型是不同的
        {

            egsl::val cov0_x, dx_dy1, dx_dy2;
            compute_covariance_exact(
                    laser_ref, laser_sens, best_x,
                    &cov0_x, &dx_dy1, &dx_dy2);

            egsl::val cov_x = egsl_->egsl_scale(square(params->sigma), cov0_x);
            // /*egsl_v2da(cov_x, res->cov_x); */

            res->cov_x_m = egsl_->egsl_v2gslm(cov_x);//修改egsl_v2gslm函数体，搞定内存泄漏BUG20201126
            res->dx_dy1_m = egsl_->egsl_v2gslm(dx_dy1);
            res->dx_dy2_m = egsl_->egsl_v2gslm(dx_dy2);

            // for(int i = 0; i < 9; i++)
            // {
            //     std::cout << "cov(x):" << res->cov_x_m->data[i] << "  ";
            // }

            //std::cout << "cov(x):" << *res->cov_x_m->data << "  " << res->cov_x_m->data[0] << std::endl;
            //std::cout << std::endl;

            if(0)
            {
                egsl_->egsl_print("cov0_x", cov0_x);
                egsl_->egsl_print_spectrum("cov0_x", cov0_x);

                //egsl::val fim = ld_fisher0(laser_ref);
                //egsl::val fim = ld_fisher(laser_ref);
                //fim = egsl_->egsl_scale(square(1.0/params->sigma), fim);//BY caiya
                // egsl::val ifim = egsl_->egsl_inverse(fim);
                // egsl_->egsl_print("fim", fim);
                // egsl_->egsl_print_spectrum("ifim", fim);
            }
        }

        res->error = best_error;
        res->iterations = iterations;
        res->nvalid = nvalid;

        gsl_vector_free(best_x);
    }
    gsl_vector_free(x_new);
    gsl_vector_free(x_old);


    egsl_->egsl_pop_named("sm_icp");
}

int CSM::icp_loop(struct sm_params*params, const double*q0, double*x_new,
		  double*total_error, int*valid, int*iterations, double* num_corres_ratio)
{
    if(any_nan(q0,3)) {
        printf("icp_loop: Initial pose contains nan: %s\n", friendly_pose(q0));
        return 0;
    }


    LDP laser_sens = params->laser_sens;
    double x_old[3], delta[3], delta_old[3] = {0,0,0};
    copy_d(q0, 3, x_old);
    unsigned int hashes[params->max_iterations];//创建N个哈西数，目的是？
    int iteration;

#ifdef _DEBUG
    printf("icp: starting at  q0 =  %s  \n", friendly_pose(x_old));
#endif


    int all_is_okay = 1;

    int num_corr_after;
      
    for(iteration=0; iteration<params->max_iterations;iteration++)
    {
        egsl_->egsl_push_named("icp_loop iteration");
	
#ifdef _DEBUG
        printf("== icp_loop: starting iteration. %d  \n", iteration);
#endif

        /** Compute laser_sens's points in laser_ref's coordinates
            by roto-translating by x_old */
        ld_compute_world_coords(laser_sens, x_old);//将laser_sens激光数据转换到相对于laser_ref坐标系下的笛卡尔坐标

        /** Find correspondences (the naif or smart way) */
        if(1)
            find_correspondences_tricks(params);//找出laser_sens集中各有效激光束对应的j1和j2(属于laser_ref集)
//        else
//            find_correspondences(params);

        /** If debug_verify_tricks, make sure that find_correspondences_tricks()
            and find_correspondences() return the same answer */

        /* If not many correspondences, bail out */
        int num_corr = ld_num_valid_correspondences(laser_sens);
        double fail_perc = 0.05;
        if(num_corr < fail_perc * laser_sens->nrays) { /* TODO: arbitrary */
            printf("	: before trimming, only %d correspondences.\n",num_corr);
            all_is_okay = 0;
            egsl_->egsl_pop_named("icp_loop iteration"); /* loop context */
            break;
        }

        /* Kill some correspondences (using dubious algorithm) */
        if(params->outliers_remove_doubles)
            kill_outliers_double(params);

        int num_corr2 = ld_num_valid_correspondences(laser_sens);

        double error=0;
        /* Trim correspondences */
        kill_outliers_trim(params, &error);

	num_corr_after = ld_num_valid_correspondences(laser_sens);

        *total_error = error;
        *valid = num_corr_after;
	
#ifdef _DEBUG
        printf("  icp_loop: total error: %f  valid %d   mean = %f\n", *total_error, *valid, *total_error/ *valid);
#endif
	
        /* If not many correspondences, bail out */
        if(num_corr_after < fail_perc * laser_sens->nrays)
        {
            printf("  icp_loop: failed: after trimming, only %d correspondences.\n",num_corr_after);
            all_is_okay = 0;
            egsl_->egsl_pop_named("icp_loop iteration"); /* loop context */
            break;
        }

        /* Compute next estimate based on the correspondences */
        if(!compute_next_estimate(params, x_old, x_new))
        {
            printf("  icp_loop: Cannot compute next estimate.\n");
            all_is_okay = 0;
            egsl_->egsl_pop_named("icp_loop iteration");
            break;
        }

        pose_diff_d(x_new, x_old, delta);

#ifdef _DEBUG
        {
            printf("  icp_loop: killing. laser_sens has %d/%d rays valid,  %d corr found -> %d after double cut -> %d after adaptive cut \n", count_equal(laser_sens->valid, laser_sens->nrays, 1), laser_sens->nrays, num_corr, num_corr2, num_corr_after);
        }
#endif
        /** Checks for oscillations */
        hashes[iteration] = ld_corr_hash(laser_sens);
#ifdef _DEBUG
        {
            printf("  icp_loop: it. %d  hash=%d nvalid=%d mean error = %f, x_new= %s\n",
                     iteration, hashes[iteration], *valid, *total_error/ *valid,
                     friendly_pose(x_new));
        }
#endif

        /** PLICP terminates in a finite number of steps! */
        if(params->use_point_to_line_distance)
        {
            int loop_detected = 0; /* TODO: make function */
            int a; for(a=iteration-1;a>=0;a--)
            {
                if(hashes[a]==hashes[iteration])
                {
#ifdef _DEBUG		  
                    printf("icpc: oscillation detected (cycle length = %d)\n", iteration-a);
#endif
                    loop_detected = 1;
                    break;
                }
            }
            if(loop_detected)
            {
                egsl_->egsl_pop_named("icp_loop iteration");
                break;
            }
        }

        /* This termination criterium is useless when using
           the point-to-line-distance; however, we put it here because
           one can choose to use the point-to-point distance. */
        if(termination_criterion(params, delta))
        {
            egsl_->egsl_pop_named("icp_loop iteration");
            break;
        }

        copy_d(x_new, 3, x_old);
        copy_d(delta, 3, delta_old);


        egsl_->egsl_pop_named("icp_loop iteration");
    }

    *iterations = iteration+1;//因爲iterations是從0開始，故此處將iterations增加1,与直觉保持一致

    *num_corres_ratio = (double)num_corr_after / params -> laser_sens -> nrays;
    
    return all_is_okay;
}

void CSM::ld_create_jump_tables(struct laser_data* ld)
{
    int i;
    for(i=0;i<ld->nrays;i++)
    {
        int j;

        j = i + 1;
        while(j<ld->nrays && ld->valid[j] && ld->readings[j]<=ld->readings[i]) j++;
        ld->up_bigger[i] = j-i;//索引往大的方向，找到第一个比第i个激光数据大的激光数据，j-i表征i跳几个单位，能到j，为正值

        j = i + 1;
        while(j<ld->nrays && ld->valid[j] && ld->readings[j]>=ld->readings[i]) j++;
        ld->up_smaller[i] = j-i;//索引往大的方向，找到第一个比第i个激光数据小的激光数据，j-i表征i跳几个单位，能到j，为正值

        j = i - 1;
        while(j>=0 && ld->valid[j] && ld->readings[j]>=ld->readings[i]) j--;
        ld->down_smaller[i] = j-i;//索引往小的方向，找到第一个比第i个激光数据小的数据，j-i表征i跳几个单位，能到j，为负值

        j = i - 1;
        while(j>=0 && ld->valid[j] && ld->readings[j]<=ld->readings[i]) j--;
        ld->down_bigger[i] = j-i;//索引往小的方向，找到第一个比第i个激光数据大的数据，j-i表征i跳几个单位，能到j，为负值
    }
}

/* Assumes that points_w is filled.  */
void CSM::find_correspondences_tricks(struct sm_params*params)
{
    const LDP laser_ref  = params->laser_ref;
    const LDP laser_sens = params->laser_sens;
    int i;

    /* Handy constant */
    double C1 =  (double)laser_ref->nrays / (laser_ref->max_theta-laser_ref->min_theta) ;
    double max_correspondence_dist2 = square(params->max_correspondence_dist);
    /* Last match */
    int last_best = -1;
    const point2d * restrict points_w = laser_sens->points_w;
    for(i=0;i<laser_sens->nrays;i++)
    {
        if(!ld_valid_ray(laser_sens,i))
        {
            ld_set_null_correspondence(laser_sens,i);//如该束激光数据无效，将其进行无效相关设置
            continue;
        }

        const double *p_i_w = points_w[i].p;//laser_sens中第i个激光束点的笛卡尔坐标
        double p_i_w_nrm2 = points_w[i].rho;
        double p_i_w_phi = points_w[i].phi;

        /** Search domain for j1 */
        int from = 0;
        int to = laser_ref->nrays-1;

        int start_cell = (int) ((p_i_w_phi - laser_ref->min_theta) * C1);

        /** Current best match */
        int j1 = -1;
        /** and his distance */
        double best_dist = 42;

        /** If last match was succesful, then start at that index + 1 */
        int we_start_at;
        if (last_best==-1)//如果是第一次匹配，则从start_cell开始匹配
            we_start_at = start_cell;
        else//如果是第二次及以后匹配，则用上一次匹配得到的对应激光束的index开始找correspondence
            we_start_at = last_best + 1;

        if(we_start_at > to) we_start_at = to;
        if(we_start_at < from) we_start_at = from;

        int up =  we_start_at+1;
        int down = we_start_at;
        double last_dist_up = 0; /* first is down */
        double last_dist_down = -1;

        int up_stopped = 0; //up方向结束搜寻标志位，为0,终止搜索
        int down_stopped = 0;



        while ( (!up_stopped) || (!down_stopped) )
        {
            int now_up = up_stopped ? 0 :
                         down_stopped ? 1 : last_dist_up < last_dist_down;

            /* Now two symmetric chunks of code, the now_up and the !now_up */
            if(now_up)
            {
                /* If we have crossed the "to" boundary we stop searching
                    on the "up" direction. */
                if(up > to)
                {
                    up_stopped = 1;
                    continue;
                }
                /* Just ignore invalid rays */
                if(!laser_ref->valid[up])
                {
                    ++up;
                    continue;
                }

                /* This is the distance from p_i_w to the "up" point*/
                last_dist_up = local_distance_squared_d(p_i_w, laser_ref->points[up].p);

                /* If it is less than the best point, it is our new j1 */
                if((last_dist_up < best_dist) || (j1==-1) )
                    j1 = up, best_dist = last_dist_up;

                /* If we are moving away from start_cell */
                if (up > start_cell)
                {
                    double delta_theta = (laser_ref->theta[up] - p_i_w_phi);
                    /* We can compute a bound for early stopping. Currently
                       our best point has distance best_dist; we can compute
                       min_dist_up, which is the minimum distance that can have
                       points for j>up (see figure)*/
                    double min_dist_up = p_i_w_nrm2 *
                                         ((delta_theta > M_PI*0.5) ? 1 : mysin(delta_theta));
                    /* If going up we can't make better than best_dist, then
                        we stop searching in the "up" direction */
                    if(square(min_dist_up) > best_dist)
                    {
                        up_stopped = 1; continue;
                    }
                    /* If we are moving away, then we can implement the jump tables
                       optimizations. */
                    up +=
                            /* If p_i_w is shorter than "up" */
                            (laser_ref->readings[up] < p_i_w_nrm2)
                            ?
                            /* We can jump to a bigger point */
                            laser_ref->up_bigger[up]
                            /* Or else we jump to a smaller point */
                            : laser_ref->up_smaller[up];

                }
                else
                    /* If we are moving towards "start_cell", we can't do any
                       ot the previous optimizations and we just move to the next point */
                    ++up;
            }

            /* This is the specular part of the previous chunk of code. */
            if(!now_up)
            {
                if(down < from) {
                    down_stopped = 1; continue; }
                if(!laser_ref->valid[down]) {
                    --down; continue; }

                last_dist_down = local_distance_squared_d(p_i_w, laser_ref->points[down].p);
                if( (last_dist_down < best_dist) || (j1==-1) )
                    j1 = down, best_dist = last_dist_down;

                if (down < start_cell) {
                    double delta_theta = (p_i_w_phi - laser_ref->theta[down]);
                    double min_dist_down = p_i_w_nrm2 *
                                           ((delta_theta > M_PI*0.5) ? 1 : mysin(delta_theta));
                    if( square(min_dist_down) > best_dist) {
                        down_stopped = 1; continue;
                    }
                    down += (laser_ref->readings[down] < p_i_w_nrm2) ?
                            laser_ref->down_bigger[down] : laser_ref->down_smaller[down];
                } else --down;
            }

        }
        /* If no point matched. */
        if( (-1==j1) || (best_dist > max_correspondence_dist2) ) {
            ld_set_null_correspondence(laser_sens, i);
            continue;
        }
        /* We ignore matching the first or the last point in the scan */
        if( 0==j1 || j1 == (laser_ref->nrays-1)) {/* no match */
            ld_set_null_correspondence(laser_sens, i);
            continue;
        }

        /* Now we want to find j2, the second best match. */
        int j2;
        /* We find the next valid point, up and down */
        int j2up   = ld_next_valid_up   (laser_ref, j1);
        int j2down = ld_next_valid_down (laser_ref, j1);
        /* And then (very boring) we use the nearest */
        if((j2up==-1)&&(j2down==-1))
        {
            ld_set_null_correspondence(laser_sens, i);
            continue;
        }
        if(j2up  ==-1) { j2 = j2down; } else
        if(j2down==-1) { j2 = j2up; } else {
            double dist_up   = local_distance_squared_d(p_i_w, laser_ref->points[j2up  ].p);
            double dist_down = local_distance_squared_d(p_i_w, laser_ref->points[j2down].p);
            j2 = dist_up < dist_down ? j2up : j2down;
        }

        last_best = j1;

        laser_sens->corr[i].valid = 1;
        laser_sens->corr[i].j1 = j1;
        laser_sens->corr[i].j2 = j2;
        laser_sens->corr[i].dist2_j1 = best_dist;
        laser_sens->corr[i].type =
                params->use_point_to_line_distance ? laser_sens->corr[i].corr_pl :
                                                     laser_sens->corr[i].corr_pp;//TODO:这么赋值，是否有问题？

    }
}

/**
	If multiple points in laser_sens match to the same point in laser_ref,
	only the nearest one wins.

	Uses: laser_sens->corr, laser_sens->p
	Modifies: laser_sens->corr
 */
void CSM::kill_outliers_double(struct sm_params*params)
{
    double threshold = 3; /* TODO: add as configurable */

    LDP laser_ref  = params->laser_ref;//注意是指针赋值，修改laser_ref就是修改params中的值
    LDP laser_sens = params->laser_sens;//注意是指针赋值，修改laser_sens就是修改params中的值

    double dist2_i[laser_sens->nrays];
    double dist2_j[laser_ref->nrays];
    int j; for(j=0;j<laser_ref->nrays;j++)
        dist2_j[j]= 1000000;

    int i;
    for(i=0;i<laser_sens->nrays;i++)
    {
        if(!ld_valid_corr(laser_sens, i)) continue;
        int j1 = laser_sens->corr[i].j1;
        dist2_i[i] = laser_sens->corr[i].dist2_j1;
        dist2_j[j1] = GSL_MIN(dist2_j[j1], dist2_i[i]);//GSL_MIN:取小者
    }//dist2_j队列中，存储的是laser_sens中某点i对应laser_ref中j1点的距离，当然，laser_sens中多点对应laser_ref中同一点j1时，
    //dist2_j[j1]就是最短的那个距离

    int nkilled = 0;
    for(i=0;i<laser_sens->nrays;i++) {
        if(!ld_valid_corr(laser_sens, i)) continue;
        int j1 = laser_sens->corr[i].j1;
        if(dist2_i[i] > (threshold*threshold)*dist2_j[j1]) //注意:此处threshold为3,即laser_sens中会存在多束激光点对应
            //laser_ref中同样一个点，那么找出laser_sens中多束激光与laser_ref中共同的j1点距离大于3×3×(x^2+y^2)的点，将对应关系
            //置为无效20201028(欧式距离也就是将大于threshold倍最短距离的对应关系置为无效)
        {
            laser_sens->corr[i].valid=0;
            nkilled ++;
        }
    }
#ifdef _DEBUG
   printf("\tkill_outliers_double: killed %d correspondences\n",nkilled);
#endif
}

/**
	Trims the corrispondences using an adaptive algorithm

	Assumes cartesian coordinates computed. (points and points_w)

	So, to disable this:
		outliers_maxPerc = 1
		outliers_adaptive_order = 1

*/
void CSM::kill_outliers_trim(struct sm_params*params,  double*total_error) //total_error是平均距离
{

    LDP laser_ref  = params->laser_ref;
    LDP laser_sens = params->laser_sens;

    /* dist2, indexed by k, contains the error for the k-th correspondence */
    int k = 0;
    double dist2[laser_sens->nrays];//dist2队列中存储的是laser_sens中每束激光与laser_ref中对应线段(j1-j2)欧式距离，

    int i;
    double dist[laser_sens->nrays];
    /* for each point in laser_sens */
    for(i=0;i<laser_sens->nrays;i++) {
        /* which has a valid correspondence */
        if(!ld_valid_corr(laser_sens, i)) { dist[i]=NAN; continue; }
        double *p_i_w = laser_sens->points_w[i].p;

        int j1 = laser_sens->corr[i].j1;
        int j2 = laser_sens->corr[i].j2;
        /* Compute the distance to the corresponding segment */
        dist[i]=  dist_to_segment_d(
                laser_ref->points[j1].p, laser_ref->points[j2].p, p_i_w);//计算laser_sens中的点到线段j1-j2的距离
        dist2[k] = dist[i];
        k++;
    }


#if 0
    double dist2_copy[k]; for(i=0;i<k;i++) dist2_copy[i] = dist2[i];
#endif

    /* two errors limits are defined: */
    /* In any case, we don't want more than outliers_maxPerc% */
    int order = (int)floor(k*(params->outliers_maxPerc));
    order = GSL_MAX(0, GSL_MIN(order, k-1));

    /* The dists for the correspondence are sorted
       in ascending order */
    quicksort(dist2, 0, k-1);//递增的方式排序dist2队列
    double error_limit1 = dist2[order];//将dist2队列中最大的数赋值给error_limit1

    /* Then we take a order statics (o*K) */
    /* And we say that the error must be less than alpha*dist(o*K) */
    int order2 = (int)floor(k*params->outliers_adaptive_order);
    order2 = GSL_MAX(0, GSL_MIN(order2, k-1));
    double error_limit2 = params->outliers_adaptive_mult*dist2[order2];//将dist2队列中最大数的
    //utliers_adaptive_mult倍赋值给error_limit1,此处params->outliers_adaptive_mult设为1是否更合理？20201028

    double error_limit = GSL_MIN(error_limit1, error_limit2);//取两者较小的数

#if 0
    double error_limit1_ho = hoare_selection(dist2_copy, 0, k-1, order);
	double error_limit2_ho = error_limit2;
	if((error_limit1_ho != error_limit1) || (error_limit2_ho != error_limit2)) {
		printf("%f == %f    %f  == %f\n",
			error_limit1_ho, error_limit1, error_limit2_ho, error_limit2);
	}
#endif
	
#ifdef _DEBUG
    printf("\ticp_outliers: maxPerc %f error_limit: fix %f adaptive %f \n",
             params->outliers_maxPerc,error_limit1,error_limit2);
#endif

    *total_error = 0;
    int nvalid = 0;
    for(i=0;i<laser_sens->nrays;i++) {
        if(!ld_valid_corr(laser_sens, i)) continue;
        if(dist[i] > error_limit) {
            laser_sens->corr[i].valid = 0;
            laser_sens->corr[i].j1 = -1;
            laser_sens->corr[i].j2 = -1;
        } else {
            nvalid++;
            *total_error += dist[i];
        }
    }
#ifdef _DEBUG
    printf("\ticp_outliers: valid %d/%d (limit: %f) mean error = %f \n",nvalid,k,error_limit,
             *total_error/nvalid);	//此处的打印出的距离，是经过平均的
#endif
    
}

int CSM::compute_next_estimate(struct sm_params*params,
                          const double x_old[3], double x_new[3])
{
    LDP laser_ref  = params->laser_ref;
    LDP laser_sens = params->laser_sens;

    struct gpc_corr c[laser_sens->nrays];//队列c中存储的是laser_sens中在laser_ref中有有效对应点的该点的相关信息

    int i; int k=0;
    for(i=0;i<laser_sens->nrays;i++)
    {
        if(!laser_sens->valid[i])
            continue;

        if(!ld_valid_corr(laser_sens,i))
            continue;

        int j1 = laser_sens->corr[i].j1;
        int j2 = laser_sens->corr[i].j2;//laser_sens中第i个对应匹配laser_ref中的两个最近邻点j1-j2

        c[k].valid = 1;

        if(laser_sens->corr[i].type == laser_sens->corr[i].corr_pl) //corr_pl为1,即用pl-icp算法
        {

            c[k].p[0] = laser_sens->points[i].p[0];
            c[k].p[1] = laser_sens->points[i].p[1];
            c[k].q[0] = laser_ref->points[j1].p[0];
            c[k].q[1] = laser_ref->points[j1].p[1];

            /** TODO: here we could use the estimated alpha */
            double diff[2];
            diff[0] = laser_ref->points[j1].p[0]-laser_ref->points[j2].p[0];
            diff[1] = laser_ref->points[j1].p[1]-laser_ref->points[j2].p[1];
            double one_on_norm = 1 / sqrt(diff[0]*diff[0]+diff[1]*diff[1]);
            double normal[2];
            normal[0] = +diff[1] * one_on_norm;
            normal[1] = -diff[0] * one_on_norm;

            double cos_alpha = normal[0];
            double sin_alpha = normal[1];

            c[k].C[0][0] = cos_alpha*cos_alpha;//C矩阵对应的就是论文中的矩阵
            c[k].C[1][0] =
            c[k].C[0][1] = cos_alpha*sin_alpha;//对称矩阵
            c[k].C[1][1] = sin_alpha*sin_alpha;

/*			sm_debug("k=%d, i=%d sens_phi: %fdeg, j1=%d j2=%d,  alpha_seg=%f, cos=%f sin=%f \n", k,i,
				rad2deg(laser_sens->theta[i]), j1,j2, atan2(sin_alpha,cos_alpha), cos_alpha,sin_alpha);*/

#if 0
            /* Note: it seems that because of numerical errors this matrix might be
			   not semidef positive. */
			double det = c[k].C[0][0] * c[k].C[1][1] - c[k].C[0][1] * c[k].C[1][0];
			double trace = c[k].C[0][0] + c[k].C[1][1];

			int semidef = (det >= 0) && (trace>0);
			if(!semidef)
			{
	/*			printf("%d: Adjusting correspondence weights\n",i);*/
				double eps = -det;
				c[k].C[0][0] += 2*sqrt(eps);
				c[k].C[1][1] += 2*sqrt(eps);
			}
#endif
        }
        else
        {
            c[k].p[0] = laser_sens->points[i].p[0];
            c[k].p[1] = laser_sens->points[i].p[1];

            projection_on_segment_d(
                    laser_ref->points[j1].p,
                    laser_ref->points[j2].p,
                    laser_sens->points_w[i].p,
                    c[k].q);

            /* Identity matrix */
            c[k].C[0][0] = 1;
            c[k].C[1][0] = 0;
            c[k].C[0][1] = 0;
            c[k].C[1][1] = 1;
        }


        double factor = 1;

        /* Scale the correspondence weight by a factor concerning the
           information in this reading. */
        if(params->use_ml_weights) //对于此处求取入射角的方法表示怀疑？以及其用处有多大？
        {
            int have_alpha = 0;
            double alpha = 0;
            if(!is_nan(laser_ref->true_alpha[j1]))
            {
                alpha = laser_ref->true_alpha[j1];
                have_alpha = 1;
            }
            else if(laser_ref->alpha_valid[j1])
            {
                alpha = laser_ref->alpha[j1];
                have_alpha = 1;
            }
            else
                have_alpha = 0;

            if(have_alpha)
            {
                double pose_theta = x_old[2];
                /** Incidence of the ray
                    Note that alpha is relative to the first scan (not the world)
                    and that pose_theta is the angle of the second scan with
                    respect to the first, hence it's ok. */
                double beta = alpha - (pose_theta + laser_sens->theta[i]);//该公式计算的是laser_sens中激光束的入射角,参考论文
                factor = 1 / square(cos(beta));//alpha是laser_sens中某束激光，对应laser_ref中的j1束激光的法向量角20201111
            }
            else
            {
                static int warned_before = 0;
                if(!warned_before)
                {
                    printf("Param use_ml_weights was active, but not valid alpha[] or true_alpha[]."
                             "Perhaps, if this is a single ray not having alpha, you should mark it as inactive.\n");
                    printf("Writing laser_ref: \n");
                    //ld_write_as_json(laser_ref, stderr);
                    warned_before = 1;
                }
            }
        }

        /* Weight the points by the sigma in laser_sens */
        if(params->use_sigma_weights)
        {
            if(!is_nan(laser_sens->readings_sigma[i]))
            {
                factor *= 1 / square(laser_sens->readings_sigma[i]);
            }
            else
            {
                static int warned_before = 0;
                if(!warned_before)
                {
                    printf("Param use_sigma_weights was active, but the field readings_sigma[] was not filled in.\n");
                    printf("Writing laser_sens: \n");
                    //ld_write_as_json(laser_sens, stderr);
                }
            }
        }

        c[k].C[0][0] *= factor;
        c[k].C[1][0] *= factor;
        c[k].C[0][1] *= factor;
        c[k].C[1][1] *= factor;

        k++;
    }

    /* TODO: use prior for odometry */
    double std = 0.11;
    const double inv_cov_x0[9] =
            {1/(std*std), 0, 0,
             0, 1/(std*std), 0,
             0, 0, 0};


    int ok = gpc_solve(k, c, 0, inv_cov_x0, x_new);
    if(!ok)
    {
        printf("gpc_solve_valid failed\n");
        return 0;
    }

    double old_error = gpc_total_error(c, k, x_old);
    double new_error = gpc_total_error(c, k, x_new);

#ifdef _DEBUG
    printf("\tcompute_next_estimate: old error: %f  x_old= %s \n", old_error, friendly_pose(x_old));
    printf("\tcompute_next_estimate: new error: %f  x_new= %s \n", new_error, friendly_pose(x_new));
    printf("\tcompute_next_estimate: new error - old_error: %g \n", new_error-old_error);
#endif
    
    double epsilon = 0.000001;
    if(new_error > old_error + epsilon)//按道理，new_error是要比old_error要小,表明是梯度下降的
    {
        printf("\tcompute_next_estimate: something's fishy here! Old error: %lf  new error: %lf  x_old %lf %lf %lf x_new %lf %lf %lf\n",old_error,new_error,x_old[0],x_old[1],x_old[2],x_new[0],x_new[1],x_new[2]);
    }

    return 1;
}

int CSM::gpc_solve(int K, const struct gpc_corr*c,
              const double*x0, const double *cov_x0,
              double *x_out)
{
    M(bigM,    4,4); M(g,  4,1); M(bigM_k,2,4);
    M(bigM_k_t,4,2); M(C_k,2,2); M(q_k,   2,1);
    M(temp41,  4,1); M(temp22,2,2);	M(temp22b,2,2);
    M(temp42,  4,2); M(temp44,4,4);	M(temp21, 2,1);
    M(temp22c, 2,2); M(temp12,1,2);

    gsl_matrix_set_zero(bigM);
    gsl_matrix_set_zero(g);
    gsl_matrix_set_zero(temp42);

    double d_bigM[4][4] = { {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}};
    double d_g[4] = {0, 0, 0, 0};
    int k;
    for(k=0;k<K;k++)
    {
        if(!c[k].valid) continue;

        double C00 = c[k].C[0][0];
        double C01 = c[k].C[0][1];
        double C10 = c[k].C[1][0];
        double C11 = c[k].C[1][1];

        if(C01 != C10) {
            fprintf(stderr, "k=%d; I expect C to be a symmetric matrix.\n", k);
            return 0;
        }
#if GPC_CHECK_SEMIDEF
        double det = C00*C11 - C01*C10;
		double trace = C00 + C11;
		int is_semidef_pos = (det >= 0) && (trace>0);
		if(!is_semidef_pos) {
			fprintf(stderr, "k=%d; I expect the matrix to be semidef positive (det>=0 and trace>0), det = %.20f trace= %.10f C = [%.15f,%.15f;%.15f %.15f]\n",
				k, det, trace,	C00,C01,C10,C11);
			return 0;
		}
#endif

        double qx = c[k].q[0];
        double qy = c[k].q[1];
        double px = c[k].p[0];
        double py = c[k].p[1];

        /* [ C00,  c01,  px C00 + c01 py , c01 px - py C00 ] */
        d_bigM[0][0] += C00;
        d_bigM[0][1] += C01;
        d_bigM[0][2] += +px * C00 + py * C01;
        d_bigM[0][3] += -py * C00 + px * C01;

        /*  [ C10 ,  C11 , py C11 + px C10 , px C11 - py C10 ] */
        d_bigM[1][0] += C10;
        d_bigM[1][1] += C11;
        d_bigM[1][2] += +px * C10 + py * C11;
        d_bigM[1][3] += +px * C11 - py * C10;

        /*Col 1 = [ py C10 + px C00 ]
         Col 2 = [ py C11 + c01 px ]
         Col 3 = [ py (py C11 + px C10) + px (px C00 + c01 py) ]
         Col 4 = [ py (px C11 - py C10) + px (c01 px - py C00) ]
        */
        d_bigM[2][0] += px * C00 + py * C10;
        d_bigM[2][1] += px * C01 + py * C11;
        d_bigM[2][2] += (px*px)*(+C00) + (px*py)*(+C10+C01) + (py*py)*(+C11);
        d_bigM[2][3] += (px*px)*(+C01) + (px*py)*(-C00+C11) + (py*py)*(-C10);

        /*Col 1 = [ px C10 - py C00 ]
          Col 2 = [ px C11 - c01 py ]
         Col 3 = [ px (py C11 + px C10) - py (px C00 + c01 py) ]
         Col 4 = [ px (px C11 - py C10) - py (c01 px - py C00) ]*/
        d_bigM[3][0] += -py * C00 + px * C10;
        d_bigM[3][1] += -py * C01 + px * C11;
        d_bigM[3][2] += (px*px)*(+C10) + (px*py)*(-C00+C11) + (py*py)*(-C01);
        d_bigM[3][3] += (px*px)*(+C11) + (px*py)*(-C10-C01) + (py*py)*(+C00);

        d_g[0] += C00*qx+C10*qy;
        d_g[1] += C01*qx+C11*qy;
        d_g[2] += qx * (C00*px+C01*py) + qy * (C10*px+C11*py);
        d_g[3] += qx * (C00*(-py)+C01*px) + qy * (C10*(-py)+C11*px);

    }

    {
        unsigned int a,b;
        for(a=0;a<4;a++)
            *gsl_matrix_ptr(g, a, 0) = -2 * d_g[a];//矩阵g
        for(a=0;a<4;a++)
            for(b=0;b<4;b++)
                gsl_matrix_set(bigM, a, b,  2 * d_bigM[a][b]);
    }


    M(mA,2,2); gms(mA,0,0, gmg(bigM,0,0)); gms(mA,0,1, gmg(bigM,0,1));
    gms(mA,1,0, gmg(bigM,1,0)); gms(mA,1,1, gmg(bigM,1,1));
    M(mB,2,2); gms(mB,0,0, gmg(bigM,0,2)); gms(mB,0,1, gmg(bigM,0,3));
    gms(mB,1,0, gmg(bigM,1,2)); gms(mB,1,1, gmg(bigM,1,3));
    M(mD,2,2); gms(mD,0,0, gmg(bigM,2,2)); gms(mD,0,1, gmg(bigM,2,3));
    gms(mD,1,0, gmg(bigM,3,2)); gms(mD,1,1, gmg(bigM,3,3));//从矩阵M获取各个子模块A B D

    M(mS,2,2); M(mSa,2,2);


    /*	mS = mD - mB.trans * mA.inv * mB;
      temp22b = inv(A) */
    egsl_->m_inv(mA, temp22b);
    /* temp22c = inv(A) * mB           */
    egsl_->m_mult(temp22b, mB, temp22c);
    /* temp22 = mB'               */
    egsl_->m_trans(mB, temp22);
    egsl_->m_mult(temp22, temp22c, temp22b);
    egsl_->m_scale(-1.0,temp22b);
    egsl_->m_add(mD,temp22b,mS);//mS矩阵存储的就是论文中的S矩阵

    /* mSa = mS.inv * mS.det; */
    egsl_->m_inv(mS, mSa);
    egsl_->m_scale(egsl_->m_det(mS), mSa);

    if(TRACE_ALGO) {
        egsl_->m_display("mA",mA);
        egsl_->m_display("mB",mB);
        egsl_->m_display("mD",mD);
        egsl_->m_display("mS",mS);
        egsl_->m_display("mSa",mSa);
    }

    M(g1,2,1);	M(g2,2,1);
    M(g1t,1,2);	M(g2t,1,2);
    M(mAi,2,2);	M(mBt,2,2);

    gms(g1,0,0,gmg(g,0,0));//g1的第一个元素等于g[0][0]
    gms(g1,1,0,gmg(g,1,0));//g1的第二个元素等于g[1][0]
    gms(g2,0,0,gmg(g,2,0));//g2的第一个元素等于g[2][0]
    gms(g2,1,0,gmg(g,3,0));//g2的第二个元素等于g[3][0]
    egsl_->m_trans(g1, g1t);//转置g1,并新存储到g1t
    egsl_->m_trans(g2, g2t);
    egsl_->m_trans(mB, mBt);
    egsl_->m_inv(mA, mAi);

    M(m1t,1,2);	M(m1,2,1);
    M(m2t,1,2);	M(m2,2,1);
    M(m3t,1,2);	M(m3,2,1);

    /* m1t = g1t*mAi*mB */
    egsl_->m_mult(g1t,mAi,temp12);
    egsl_->m_mult(temp12,mB,m1t);

    egsl_->m_trans(m1t,m1);
    /*     m2t = m1t*mSa    */
    egsl_->m_mult(m1t,mSa,m2t);
    egsl_->m_trans(m2t,m2);
    /* m3t = g2t*mSa     */
    egsl_->m_mult(g2t,mSa,m3t);
    egsl_->m_trans(m3t,m3);

    double p[3] = {
            egsl_->m_dot(m2t,m2) - 2*egsl_->m_dot(m2t,m3) + egsl_->m_dot(m3t,m3),
            4*egsl_->m_dot(m2t,m1) - 8*egsl_->m_dot(m2t,g2) + 4*egsl_->m_dot(g2t,m3),
            4*egsl_->m_dot(m1t,m1) - 8*egsl_->m_dot(m1t,g2) + 4*egsl_->m_dot(g2t,g2)};

    double l[3] = {egsl_->m_det(mS), 2*gmg(mS,0,0)+2*gmg(mS,1,1), 4};

    /* q = p - l^2        */
    double q[5] = {p[0]-(l[0]*l[0]), p[1]-(2*l[1]*l[0]),
                   p[2]-(l[1]*l[1]+2*l[0]*l[2]), -(2*l[2]*l[1]), -(l[2]*l[2])};

    if(TRACE_ALGO)
    {
        fprintf(stderr, "p = %f %f %f \n", p[2], p[1], p[0]);
        fprintf(stderr, "l = %f %f %f \n", l[2], l[1], l[0]);
        fprintf(stderr, "q = %f %f %f %f %f \n", q[4],  q[3],  q[2], q[1], q[0]);
    }

    double lambda;
    if(!egsl_->poly_greatest_real_root(5, q, &lambda)) return 0;

    M(W,4,4); gsl_matrix_set_zero(W); gms(W,2,2,1.0); gms(W,3,3,1.0);
    M(x,4,1);

    egsl_->m_scale(2*lambda, W);
    gsl_matrix_add(bigM,W);
    egsl_->m_inv(bigM, temp44);
    egsl_->m_mult(temp44, g, x);
    egsl_->m_scale(-1.0, x);

    x_out[0] = gmg(x,0,0);
    x_out[1] = gmg(x,1,0);
    x_out[2] = atan2(gmg(x,3,0),gmg(x,2,0));

    if(TRACE_ALGO) {
        fprintf(stderr, "x =  %f  %f %f deg\n", x_out[0], x_out[1],x_out[2]*180/M_PI);
    }

    MF(mA); MF(mB); MF(mD); MF(mS); MF(mSa);
    MF(m1t);	MF(m1);	MF(m2t);	MF(m2);
    MF(m3t);	MF(m3);	MF(W); 	MF(x);
    MF(bigM); MF(g); MF(bigM_k);
    MF(bigM_k_t); MF(C_k); MF(q_k);
    MF(temp42); MF(temp44);	MF(temp21);
    MF(temp41); MF(temp22);	MF(temp22b);
    MF(temp22c); MF(temp12);
    MF(g1);	MF(g2);
    MF(g1t);	MF(g2t);
    MF(mAi);	MF(mBt);
    return 1;
}

double CSM::gpc_total_error(const struct gpc_corr*co, int n, const double*x)
{
    double error = 0;
    for(int i=0;i<n;i++) {
        if(!co[i].valid) continue;
        error += gpc_error(co+i,x);
    }
    if(0) /* due to limited numerical precision, error might be negative */
        if(error<0) {
            fprintf(stderr, "Something fishy!\n");
        }
    return error;
}

double CSM::gpc_error(const struct gpc_corr*co, const double*x)
{
    double c = cos(x[2]);
    double s = sin(x[2]);
    double e[2];
    e[0] = c*(co->p[0]) -s*(co->p[1]) + x[0] - co->q[0];
    e[1] = s*(co->p[0]) +c*(co->p[1]) + x[1] - co->q[1];
    double this_error = e[0]*e[0]*co->C[0][0]+2*e[0]*e[1]*co->C[0][1]+e[1]*e[1]*co->C[1][1];

    if(0) /* due to limited numerical precision, error might be negative */
        if(this_error < 0) {
            fprintf(stderr, "Something fishy: error = %lf e = [%lf %lf]  C = [%lf,%lf;%lf,%lf]\n", this_error,
                    e[0],e[1],co->C[0][0],co->C[0][1],co->C[1][0],co->C[1][1]);
        }
    return this_error;
}

int CSM::termination_criterion(struct sm_params*params, const double*delta)
{
    double a = norm_d(delta);
    double b = fabs(delta[2]);
    return (a<params->epsilon_xy) && (b<params->epsilon_theta);
}

/* computer covariance function*/
void CSM::compute_covariance_exact(
        LDP laser_ref, LDP laser_sens, const gsl_vector*x,
        egsl::val *cov0_x, egsl::val *dx_dy1, egsl::val *dx_dy2)
{
    egsl_->egsl_push_named("compute_covariance_exact");

    egsl::val d2J_dxdy1 = egsl_->egsl_zeros(3,(size_t)laser_ref ->nrays);
    egsl::val d2J_dxdy2 = egsl_->egsl_zeros(3,(size_t)laser_sens->nrays);

    /* the three pieces of d2J_dx2 */
    egsl::val d2J_dt2       = egsl_->egsl_zeros(2,2);
    egsl::val d2J_dt_dtheta = egsl_->egsl_zeros(2,1);
    egsl::val d2J_dtheta2   = egsl_->egsl_zeros(1,1);

    double theta = x->data[2];
    egsl::val t = egsl_->egsl_vFa(2,x->data);

    int i;
    for(i=0;i<laser_sens->nrays;i++) 
    {
        if(!ld_valid_corr(laser_sens,i)) continue;
        egsl_->egsl_push_named("compute_covariance_exact iteration");

        int j1 = laser_sens->corr[i].j1;
        int j2 = laser_sens->corr[i].j2;

        egsl::val p_i  = egsl_->egsl_vFa(2, laser_sens->points[i].p);
        egsl::val p_j1 = egsl_->egsl_vFa(2, laser_ref ->points[j1].p);
        egsl::val p_j2 = egsl_->egsl_vFa(2, laser_ref ->points[j2].p);

        /* v1 := rot(theta+M_PI/2)*p_i */
        egsl::val v1 = egsl_->egsl_mult(egsl_->egsl_rot(theta+M_PI/2), p_i);
        /* v2 := (rot(theta)*p_i+t-p_j1) */
        //egsl::val v2 = sum3( m(egsl_->egsl_rot(theta),p_i), t, minus(p_j1));
        egsl::val v2 = egsl_->egsl_sum( egsl_->egsl_mult(egsl_->egsl_rot(theta),p_i), egsl_->egsl_sum(t, egsl_->egsl_scale(-1.0,p_j1)));
        /* v3 := rot(theta)*v_i */
        egsl::val v3 = egsl_->egsl_vers(theta + laser_sens->theta[i]);//此处rho_i是极半径的意思，激光束i表示为rho_i(laser_sens_theta)
        /* v4 := rot(theta+PI/2)*v_i; */
        egsl::val v4 = egsl_->egsl_vers(theta + laser_sens->theta[i] + M_PI/2);

        egsl::val C_k = compute_C_k(p_j1, p_j2);

        egsl::val d2J_dt2_k = egsl_->egsl_scale(2.0, C_k);//误差函数J(k)对t的二阶偏导
        egsl::val d2J_dt_dtheta_k = egsl_->egsl_scale(2.0,egsl_->egsl_mult(C_k,v1));//误差函数J(k)对t和theta的二阶导数

        egsl::val v_new = egsl_->egsl_mult(egsl_->egsl_rot(theta+M_PI), p_i);
        //egsl::val d2J_dtheta2_k = sc(2.0, sum( m3(tr(v2),C_k, v_new), m3(tr(v1),C_k,v1)));//误差函数对theta^2的二阶导数
        egsl::val d2J_dtheta2_k = egsl_->egsl_scale(2.0, egsl_->egsl_sum( egsl_->egsl_mult(egsl_->egsl_transpose(v2),egsl_->egsl_mult(C_k, v_new)),
        egsl_->egsl_mult(egsl_->egsl_transpose(v1),egsl_->egsl_mult(C_k, v_new))));

        egsl_->egsl_add_to(d2J_dt2, d2J_dt2_k);//将每个对应关系下的误差函数J_i的关于t的二阶偏导累加，累加到d2J_dt2上去
        egsl_->egsl_add_to(d2J_dt_dtheta, d2J_dt_dtheta_k ); //同理
        egsl_->egsl_add_to(d2J_dtheta2, d2J_dtheta2_k);

        /* for measurement rho_i  in the second scan */
        egsl::val d2Jk_dtdrho_i = egsl_->egsl_scale(2.0, egsl_->egsl_mult(C_k,v3)); //2x1大小的矩阵
        //egsl::val d2Jk_dtheta_drho_i = egsl_->egsl_scale(2.0, sum( m3(tr(v2),C_k,v4),  m3(tr(v3),C_k,v1)));//1x1大小矩阵

        egsl::val d2Jk_dtheta_drho_i = egsl_->egsl_scale(2.0, egsl_->egsl_sum( egsl_->egsl_mult(egsl_->egsl_transpose(v2),egsl_->egsl_mult(C_k,v4)),
                                                                               egsl_->egsl_mult(egsl_->egsl_transpose(v3),egsl_->egsl_mult(C_k,v1))));

        egsl_->egsl_add_to_col(d2J_dxdy2, (size_t)i, egsl_->egsl_compose_col(d2Jk_dtdrho_i, d2Jk_dtheta_drho_i));//将解算出来的两个偏导项累加到
        //d2J_dxdy2的第i列

        /* for measurements rho_j1, rho_j2 in the first scan */

        egsl::val dC_drho_j1 = dC_drho(p_j1, p_j2);
        egsl::val dC_drho_j2 = dC_drho(p_j2, p_j1);


        egsl::val v_j1 = egsl_->egsl_vers(laser_ref->theta[j1]);

        egsl::val d2Jk_dt_drho_j1 = egsl_->egsl_sum(egsl_->egsl_scale(-2.0,egsl_->egsl_mult(C_k,v_j1)), egsl_->egsl_scale(2.0,egsl_->egsl_mult(dC_drho_j1,v2)));//2x1大小
        //egsl::val d2Jk_dtheta_drho_j1 = sum( sc(-2.0, m3(tr(v_j1),C_k,v1)), m3(tr(v2),dC_drho_j1,v1));//1x1大小

        egsl::val d2Jk_dtheta_drho_j1 = egsl_->egsl_sum( egsl_->egsl_scale(-2.0, egsl_->egsl_mult(egsl_->egsl_transpose(v_j1),egsl_->egsl_mult(C_k,v1))),
                                             egsl_->egsl_mult(egsl_->egsl_transpose(v2),egsl_->egsl_mult(dC_drho_j1,v1)));

        egsl_->egsl_add_to_col(d2J_dxdy1, (size_t)j1, egsl_->egsl_compose_col(d2Jk_dt_drho_j1, d2Jk_dtheta_drho_j1));

        /* for measurement rho_j2*/
        egsl::val d2Jk_dt_drho_j2 = egsl_->egsl_scale(2.0, egsl_->egsl_mult( dC_drho_j2,v2));
        egsl::val d2Jk_dtheta_drho_j2 = egsl_->egsl_scale(2.0, egsl_->egsl_mult( egsl_->egsl_transpose(v2), egsl_->egsl_mult(dC_drho_j2, v1)));
        egsl_->egsl_add_to_col(d2J_dxdy1, (size_t)j2, egsl_->egsl_compose_col(d2Jk_dt_drho_j2, d2Jk_dtheta_drho_j2));

        egsl_->egsl_pop_named("compute_covariance_exact iteration");
    }

    /*composes matrix  d2J_dx2  from the pieces*/
    egsl::val d2J_dx2   = egsl_->egsl_compose_col( egsl_->egsl_compose_row(d2J_dt2, d2J_dt_dtheta),
                                    egsl_->egsl_compose_row(egsl_->egsl_transpose(d2J_dt_dtheta), d2J_dtheta2));

    egsl::val edx_dy1 = egsl_->egsl_scale(-1.0, egsl_->egsl_mult(egsl_->egsl_inverse(d2J_dx2), d2J_dxdy1));
    egsl::val edx_dy2 = egsl_->egsl_scale(-1.0, egsl_->egsl_mult(egsl_->egsl_inverse(d2J_dx2), d2J_dxdy2));

    egsl::val ecov0_x = egsl_->egsl_sum(egsl_->egsl_mult(edx_dy1,egsl_->egsl_transpose(edx_dy1)),egsl_->egsl_mult(edx_dy2,egsl_->egsl_transpose(edx_dy2)) );

    /* With the egsl_promote we save the matrix in the previous
       context */
    *cov0_x = egsl_->egsl_promote(ecov0_x);
    *dx_dy1 = egsl_->egsl_promote(edx_dy1);
    *dx_dy2 = egsl_->egsl_promote(edx_dy2);

    egsl_->egsl_pop_named("compute_covariance_exact");
    /* now edx_dy1 is not valid anymore, but *dx_dy1 is. */
}

egsl::val CSM::compute_C_k(egsl::val p_j1, egsl::val p_j2)  {
    egsl::val d = egsl_->egsl_sub(p_j1, p_j2);
    double alpha = M_PI/2 + atan2( egsl_->egsl_atv(d,1), egsl_->egsl_atv(d,0));
    double c = cos(alpha); double s = sin(alpha);
    double m[2*2] = {
            c*c, c*s,
            c*s, s*s
    };
    return egsl_->egsl_vFda(2,2,m);
}

egsl::val CSM::dC_drho(egsl::val p1, egsl::val p2) {
    double eps = 0.001;

    egsl::val C_k = compute_C_k(p1, p2);
    egsl::val p1b = egsl_->egsl_sum(p1, egsl_->egsl_scale(eps/egsl_->egsl_norm(p1),p1));
    egsl::val C_k_eps = compute_C_k(p1b,p2);
    return egsl_->egsl_scale(1/eps, egsl_->egsl_sub(C_k_eps,C_k));
}

egsl::val CSM::ld_fisher0(LDP ld) 
{
    egsl_->egsl_push_named("calculate fim");
    egsl::val fim = egsl_->egsl_zeros(3,3);
    int i;
    for(i=0;i<ld->nrays;i++) 
    {
        //double alpha = ld->true_alpha[i];//Censi的写法
        double alpha = ld->alpha[i];//my personal algorithm,
        if(is_nan(alpha)) continue;

        double theta = ld->theta[i];
        double beta  = alpha-theta;//計算單個激光數據集時，此時假設激光的姿態角是0度!!!此處的theta是順時針還是逆時針？分正負？
        //我認爲作者這麼寫，是有問題的，因爲入射角beta應該是0-90度範圍,且是正的

        double r = ld->readings[i];
        double c = cos(alpha);
        double s = sin(alpha);

        double z = 1 / cos(beta);
        double t = tan(beta);

        double fim_k[9] ={
                c*c*z*z,    c*s*z*z,   c*z *t*r,
                c*s*z*z,    s*s*z*z,   s*z *t*r,
                c*z*t*r,    s*z*t*r,   t*r *t*r
        };

        egsl_->egsl_push();
        egsl::val k = egsl_->egsl_vFda(3,3,fim_k);
        egsl_->egsl_add_to(fim, k);
        egsl_->egsl_pop();
    }
    egsl_->egsl_print("fim", fim);
    egsl_->egsl_print_spectrum("ifim", fim);

    egsl_->egsl_pop_named("calculate fim");

    return fim;
}

egsl::val CSM::ld_fisher(LDP ld)//University of California at Berkeley's solution 
{
    egsl_->egsl_push_named("calculate fim");
    
    egsl::val fim = egsl_->egsl_zeros(2,2);
    int i;
    for(i=0;i<ld->nrays;i++) 
    {
        //double alpha = ld->true_alpha[i];//Censi的写法
        double alpha = ld->alpha[i];//my personal algorithm,
        if(is_nan(alpha)) continue;

        
        
        double c = cos(alpha);
        double s = sin(alpha);

     
        double fim_k[4] ={
                c*c,    c*s,   
                c*s,    s*s       
                };

        egsl_->egsl_push();
        egsl::val k = egsl_->egsl_vFda(2,2,fim_k);
        egsl_->egsl_add_to(fim, k);
        egsl_->egsl_pop();
    }
    
#ifdef _DEBUG
    egsl_->egsl_print("fim", fim);
    egsl_->egsl_print_spectrum("ifim", fim);
#endif
    
    egsl_->egsl_pop_named("calculate fim");

    return fim;
}
