#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string groundtruth_file = "../groundtruth.txt";
string estimate_file = "../estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_vec;
void DrawTrajectory(pose_vec, pose_vec);

int main(int argc, char **argv) {

    pose_vec gt_poses, est_poses;

    /// implement pose reading code
    ifstream f_gt(groundtruth_file);
    while(!f_gt.eof()) {
        Eigen::Vector3d t_tra;
        double t, t_x, t_y, t_z, q_x, q_y, q_z, q_w;
        f_gt >> t >> t_x >> t_y >> t_z;
        t_tra << t_x, t_y, t_z;
        f_gt >> q_x >> q_y >> q_z >> q_w;
        Eigen::Quaterniond q(q_w, q_x, q_y, q_z);
        gt_poses.emplace_back(Sophus::SE3(q,t_tra));
    }

    ifstream f_est(estimate_file);
    while(!f_est.eof()) {
        Eigen::Vector3d t_tra;
        double t, t_x, t_y, t_z, q_x, q_y, q_z, q_w;
        f_est >> t >> t_x >> t_y >> t_z;
        t_tra << t_x, t_y, t_z;
        f_est >> q_x >> q_y >> q_z >> q_w;
        Eigen::Quaterniond q(q_w, q_x, q_y, q_z);
        est_poses.emplace_back(Sophus::SE3(q,t_tra));
    }

    // draw trajectory in pangolin
    DrawTrajectory(gt_poses, est_poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(pose_vec gt_poses, pose_vec est_poses) {
    if (est_poses.empty()) {
        cerr << "estimated is empty!" << endl;
        return;
    }
    if (gt_poses.empty()) {
        cerr << "groundtruth is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < gt_poses.size() - 1; i++) {
            glColor3f(1 - (float) i / gt_poses.size(), 0.0f, (float) i / gt_poses.size());
            glBegin(GL_LINES);
            auto p1 = gt_poses[i], p2 = gt_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < est_poses.size() - 1; i++) {
            glColor3f(1 - (float) i / est_poses.size(), 0.0f, (float) i / est_poses.size());
            glBegin(GL_LINES);
            auto p1 = est_poses[i], p2 = est_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}