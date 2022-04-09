// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#ifndef KIN_MODEL_H
#define KIN_MODEL_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include "Eigen/Dense"
#include <tinyxml.h>
#include <map>
#include <list>
#include <vector>
#include <set>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

class KinematicModel {
public:

    typedef Eigen::MatrixXd Jacobian;

    KinematicModel(const std::string &urdf_string, const std::vector<std::string > &joint_names);
    ~KinematicModel();

    void calculateFkAll(const Eigen::VectorXd &q);
    KDL::Frame getFrame(const std::string &name) const;

    void getJacobian(Jacobian &jac, const std::string &link_name, const Eigen::VectorXd &q) const;
    void calculateFk(KDL::Frame &T, const std::string &link_name, const Eigen::VectorXd &q) const;
    void calculateFk(KDL::Frame &T, const std::string &link_name, const Eigen::VectorXd &q, const Eigen::VectorXd &ign_q) const;
    void getJacobiansForPairX(Jacobian &jac1, Jacobian &jac2,
                                        const std::string &link_name1, const KDL::Vector &x1,
                                        const std::string &link_name2, const KDL::Vector &x2, const Eigen::VectorXd &q) const;

    void setIgnoredJointValue(const std::string &joint_name, double value);
    //void setIgnoredJointValues(const std::vector<std::string > &joint_names, const Eigen::VectorXd &q);
    void setIgnoredJointValue(unsigned int idx, double value);
    //void getIgnoredJoints(Eigen::VectorXd &ign_q, std::vector<std::string > &ign_joint_names) const;
    void getIgnoredJointsNameVector(std::vector<std::string > &joint_name_vec) const;

    double getLowerLimit(int q_idx) const;
    double getUpperLimit(int q_idx) const;

    bool getJointLinkName(int q_idx, std::string &link_name) const;
    bool getJointAxisAndOrigin(int q_idx, KDL::Vector &axis, KDL::Vector &origin) const;

    int getDofCount() const;

    int getJointIndex(const std::string &joint_name) const;
    int getIgnoredJointIndex(const std::string &joint_name) const;

    const std::vector<std::string > &getJointNames() const;

    int getJointCount() const;
    int getIgnoredJointCount() const;

    bool getSubtreeLinks(const std::string &root_name, std::list<std::string > &link_names) const;

protected:
    class Mimic {
    public:
        int q_nr_;
        double multiplier_;
        double offset_;
    };

    void setLowerLimit(int q_idx, double limit);
    void setUpperLimit(int q_idx, double limit);

    void getJointValuesKDL(const Eigen::VectorXd &q, KDL::JntArray &q_kdl) const;
    void getJointValuesKDL(const Eigen::VectorXd &q, const Eigen::VectorXd &ign_q, KDL::JntArray &q_kdl) const;

    void getJacobianForX(Jacobian &jac, const std::string &link_name, const KDL::Vector &x, const KDL::JntArray &q_kdl, const std::string &base_name) const;

    bool parseMimic(std::string &mimic_name, double &multiplier, double &offset, TiXmlElement* o);
    bool parseLimit(double &lower_limit, double &upper_limit, TiXmlElement* o);
    bool parseJoint(TiXmlElement* o);
    bool parseURDF(const std::string &xml_string);

    KDL::Tree tree_;
    std::map<int, int > q_idx_q_nr_map_, q_nr_q_idx_map_;
//    std::map<int, int > ign_q_idx_q_nr_map_, ign_q_nr_q_idx_map_;
    std::vector<int > ign_q_idx_q_nr_vec_, ign_q_nr_q_idx_vec_;
    boost::shared_ptr<KDL::TreeJntToJacSolver > pjac_solver_;
    boost::shared_ptr<KDL::TreeFkSolverPos_recursive > pfk_solver_;
    std::map<std::string, int > ign_joint_name_q_idx_map_;
    std::map<std::string, int > joint_name_q_nr_map_;
    Eigen::VectorXd ign_q_;
    std::vector<boost::shared_ptr<Mimic > > q_nr_joint_mimic_vec_;
//    std::map<int, boost::shared_ptr<Mimic > > q_nr_joint_mimic_map_;
    std::map<std::string, double> joint_lower_limit_;
    std::map<std::string, double> joint_upper_limit_;

    std::vector<double> joint_lower_limit_q_idx_;
    std::vector<double> joint_upper_limit_q_idx_;

    std::vector<std::string > joint_names_;
    std::map<int, std::string> q_idx_link_name_map_;
    int ndof_;

    mutable KDL::JntArray q_in_;
    mutable KDL::Jacobian jac_out_;
    mutable std::vector<int > vec_int_links_;
    mutable KDL::JntArray q_kdl_;

    std::vector<const KDL::TreeElement* > sorted_seg_;
    std::vector<int > sorted_seg_parent_idx_;
    std::vector<KDL::Frame > fk_frames_;
};

#endif

