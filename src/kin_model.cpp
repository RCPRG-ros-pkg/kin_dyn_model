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

#include "kin_model/kin_model.h"

#include <set>

KinematicModel::KinematicModel(const std::string &urdf_string, const std::vector<std::string > &joint_names)
{
    if (!kdl_parser::treeFromString(urdf_string, tree_)){
        std::cout << "Failed to construct kdl tree" << std::endl;
        return;
    }

    pjac_solver_.reset(new KDL::TreeJntToJacSolver(tree_));
    pfk_solver_.reset(new KDL::TreeFkSolverPos_recursive(tree_));

    for (KDL::SegmentMap::const_iterator seg_it = tree_.getSegments().begin(); seg_it != tree_.getSegments().end(); seg_it++) {
        const std::string &joint_name = seg_it->second.segment.getJoint().getName();
//        std::cout << seg_it->first << " " << joint_name << "  q_nr: " << seg_it->second.q_nr << "  type: " << seg_it->second.segment.getJoint().getType() << std::endl;
        bool found_joint = false;
        for (int q_idx = 0; q_idx < joint_names.size(); q_idx++) {
            if (joint_names[q_idx] == joint_name) {
                if (seg_it->second.segment.getJoint().getType() == KDL::Joint::None) {
                    std::cout << "ERROR: KinematicModel::KinematicModel: joint " << joint_name << " has type: None" << std::endl;
                }
                std::cout << "mapping joint " << joint_name << " q_idx=" << q_idx << "  q_nr=" << seg_it->second.q_nr << std::endl;
                q_idx_q_nr_map_.insert( std::make_pair(q_idx, seg_it->second.q_nr) );
                q_nr_q_idx_map_.insert( std::make_pair(seg_it->second.q_nr, q_idx) );
                found_joint = true;
                break;
            }
        }
        if (!found_joint && seg_it->second.segment.getJoint().getType() != KDL::Joint::None) {
            int q_idx = ign_joint_name_q_idx_map_.size();
            std::cout << "mapping ignored joint " << joint_name << " q_idx=" << q_idx << "  q_nr=" << seg_it->second.q_nr << std::endl;
            ign_q_idx_q_nr_map_.insert( std::make_pair(q_idx, seg_it->second.q_nr) );
            ign_q_nr_q_idx_map_.insert( std::make_pair(seg_it->second.q_nr, q_idx) );

            ign_joint_name_q_idx_map_.insert( std::make_pair(joint_name, q_idx) );
        }

        if (seg_it->second.segment.getJoint().getType() != KDL::Joint::None) {
            joint_name_q_nr_map_[joint_name] = seg_it->second.q_nr;
        }
    }
    ign_q_.resize(ign_joint_name_q_idx_map_.size());
    for (int q_idx = 0; q_idx < ign_joint_name_q_idx_map_.size(); q_idx++) {
        ign_q_(q_idx) = 0.0;
    }

//    *((int*)0) = 1;
    parseURDF(urdf_string);

    for (int q_idx = 0; q_idx < joint_names.size(); q_idx++) {
        std::map<std::string, double>::const_iterator it_lo = joint_lower_limit_.find(joint_names[q_idx]);
        if (it_lo == joint_lower_limit_.end()) {
            std::cout << "ERROR: KinematicModel::KinematicModel joint " << joint_names[q_idx] << " not found in joint_lower_limit_ map" << std::endl;
            return;
        }

        std::map<std::string, double>::const_iterator it_up = joint_upper_limit_.find(joint_names[q_idx]);
        if (it_up == joint_upper_limit_.end()) {
            std::cout << "ERROR: KinematicModel::KinematicModel joint " << joint_names[q_idx] << " not found in joint_upper_limit_ map" << std::endl;
            return;
        }

        joint_lower_limit_q_idx_.push_back(it_lo->second);
        joint_upper_limit_q_idx_.push_back(it_up->second);
    }


    for (KDL::SegmentMap::const_iterator seg_it = tree_.getSegments().begin(); seg_it != tree_.getSegments().end(); seg_it++) {
        const std::string &joint_name = seg_it->second.segment.getJoint().getName();
        for (int q_idx = 0; q_idx < joint_names.size(); q_idx++) {
            if (joint_name == joint_names[q_idx]) {
                const std::string &link_name = seg_it->second.segment.getName();
                q_idx_link_name_map_[q_idx] = link_name;
            }
        }
    }

    ndof_ = joint_names.size();
}

bool KinematicModel::parseMimic(std::string &mimic_name, double &multiplier, double &offset, TiXmlElement* o)
{
	const char *joint_char = o->Attribute("joint");
	const char *multiplier_char = o->Attribute("multiplier");
	const char *offset_char = o->Attribute("offset");

	if (!joint_char)
	{
		std::cout << "No mimic name given for the joint." << std::endl;
		return false;
	}
    else {
        mimic_name = joint_char;
    }

	if (multiplier_char)
	{
        try
        {
            multiplier = boost::lexical_cast<double>(multiplier_char);
        }
        catch (boost::bad_lexical_cast &e)
        {
            std::cout << "Joint mimic multiplier (" << multiplier_char << ") is not a valid float" << std::endl;
            return false;
        }
	}
    else {
        multiplier = 1.0;
    }
	if (offset_char)
	{
        try
        {
            offset = boost::lexical_cast<double>(offset_char);
        }
        catch (boost::bad_lexical_cast &e)
        {
            std::cout << "Joint mimic offset (" << offset_char << ") is not a valid float" << std::endl;
            return false;
        }
	}
    else {
        offset = 0.0;
    }

    return true;
}

bool KinematicModel::parseLimit(double &lower_limit, double &upper_limit, TiXmlElement* o)
{
// e.g. <limit effort="100" lower="-2.792444444" upper="2.792444444" velocity="100"/>

	const char *lower_char = o->Attribute("lower");
	const char *upper_char = o->Attribute("upper");
	if (lower_char)
	{
        try
        {
            lower_limit = boost::lexical_cast<double>(lower_char);
        }
        catch (boost::bad_lexical_cast &e)
        {
            std::cout << "Joint lower limit (" << lower_char << ") is not a valid float" << std::endl;
            return false;
        }
	}
    else {
        lower_limit = 0.0;
    }
	if (upper_char)
	{
        try
        {
            upper_limit = boost::lexical_cast<double>(upper_char);
        }
        catch (boost::bad_lexical_cast &e)
        {
            std::cout << "Joint upper limit (" << upper_char << ") is not a valid float" << std::endl;
            return false;
        }
	}
    else {
        upper_limit = 0.0;
    }

    return true;
}

bool KinematicModel::parseJoint(TiXmlElement* o)
{
	const char *name_char = o->Attribute("name");
	if (!name_char)
	{
		std::cout << "No name given for the joint." << std::endl;
		return false;
	}
	std::string joint_name = std::string(name_char);

    // joint limit is optional
	TiXmlElement *lim = o->FirstChildElement("limit");
	if (lim) {
        double lower_limit, upper_limit;
		if (!parseLimit(lower_limit, upper_limit, lim)) {
			return false;
        }

        joint_lower_limit_[joint_name] = lower_limit;
        joint_upper_limit_[joint_name] = upper_limit;
	}

    // joint mimic is optional
	TiXmlElement *mim = o->FirstChildElement("mimic");
	if (mim) {
        std::string mimic_name;
        double multiplier=0.0, offset=0.0;
		if (!parseMimic(mimic_name, multiplier, offset, mim)) {
			return false;
        }

        boost::shared_ptr<Mimic > mimic(new Mimic);
        mimic->multiplier_ = multiplier;
        mimic->offset_ = offset;
        std::map<std::string, int >::const_iterator m_it = joint_name_q_nr_map_.find(mimic_name);
        if (m_it == joint_name_q_nr_map_.end()) {
            std::cout << "ERROR: could not find mimic joint " << mimic_name << std::endl;
            return false;
        }
        std::map<std::string, int >::const_iterator j_it = joint_name_q_nr_map_.find(joint_name);
        if (j_it == joint_name_q_nr_map_.end()) {
            std::cout << "ERROR: could not find joint " << joint_name << std::endl;
            return false;
        }
        mimic->q_nr_ = m_it->second;
        q_nr_joint_mimic_map_[j_it->second] = mimic;
	}

    return true;
}

bool KinematicModel::parseURDF(const std::string &xml_string)
{
	TiXmlDocument xml_doc;
	xml_doc.Parse(xml_string.c_str());
	if (xml_doc.Error())
	{
		std::cout << xml_doc.ErrorDesc() << std::endl;
		xml_doc.ClearError();
		return false;
	}
	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	if (!robot_xml)
	{
		std::cout << "Could not find the 'robot' element in the xml file" << std::endl;
		return false;
	}

	// Get all Joint elements
	for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
	{
		try {
			parseJoint(joint_xml);
		}
		catch (urdf::ParseError &e) {
			std::cout << "joint xml is not initialized correctly" << std::endl;
			return false;
		}
	}

	return true;
}

void KinematicModel::setIgnoredJointValue(const std::string &joint_name, double value) {
    std::map<std::string, int >::iterator it = ign_joint_name_q_idx_map_.find(joint_name);
    if (it == ign_joint_name_q_idx_map_.end()) {
        std::cout << "ERROR: KinematicModel::setIgnoredJointValue: wrong joint name: " << joint_name << std::endl;
        return;
    }

    std::map<std::string, int >::const_iterator j_it = joint_name_q_nr_map_.find(joint_name);
    if (j_it == joint_name_q_nr_map_.end()) {
        std::cout << "ERROR: KinematicModel::setIgnoredJointValue: could not find joint: " << joint_name << std::endl;
        return;
    }
    int q_nr = j_it->second;
    std::map<int, boost::shared_ptr<Mimic > >::const_iterator m_it = q_nr_joint_mimic_map_.find(q_nr);
    if (m_it != q_nr_joint_mimic_map_.end()) {
        std::cout << "ERROR: KinematicModel::setIgnoredJointValue: trying to set value of mimic joint: " << joint_name << std::endl;
        return;
    }
    ign_q_(it->second) = value;
//    std::cout << "KinematicModel::setIgnoredJointValue joint " << it->second << " value " << value << " " << ign_q_(it->second) << std::endl;
}

void KinematicModel::getIgnoredJoints(Eigen::VectorXd &q, std::vector<std::string > &ign_joint_names) const {
    q.resize(ign_joint_name_q_idx_map_.size());
    ign_joint_names.clear();

    int ret_q_idx = 0;
    for (std::map<std::string, int >::const_iterator it = ign_joint_name_q_idx_map_.begin(); it != ign_joint_name_q_idx_map_.end(); it++, ret_q_idx++) {
        int ign_q_idx = it->second;
        ign_joint_names.push_back(it->first);
        int qj_nr = ign_q_idx_q_nr_map_.find(ign_q_idx)->second;
        std::map<int, boost::shared_ptr<Mimic > >::const_iterator j_it = q_nr_joint_mimic_map_.find(qj_nr);
        if (j_it != q_nr_joint_mimic_map_.end()) {
            int ign_qm_idx = ign_q_nr_q_idx_map_.find(j_it->second->q_nr_)->second;
            q(ret_q_idx) = ign_q_(ign_qm_idx) * j_it->second->multiplier_ + j_it->second->offset_;
        }
        else {
            q(ret_q_idx) = ign_q_(ign_q_idx);
        }
    }

    // update mimic joints
//    for (std::map<int, boost::shared_ptr<Mimic > >::const_iterator j_it = q_nr_joint_mimic_map_.begin(); j_it != q_nr_joint_mimic_map_.end(); j_it++) {
//        int qj_idx  = ign_q_nr_q_idx_map_.find(j_it->first)->second;
//        int qm_idx = ign_q_nr_q_idx_map_.find(j_it->second->q_nr_)->second;
//        q(qj_idx) = q(qm_idx) * j_it->second->multiplier_ + j_it->second->offset_;
//    }

}

KinematicModel::~KinematicModel() {
}

void KinematicModel::getJacobian(Jacobian &jac, const std::string &link_name, const Eigen::VectorXd &q) const {
    KDL::JntArray q_in( tree_.getNrOfJoints() );
    getJointValuesKDL(q, q_in);

    KDL::Jacobian jac_out( tree_.getNrOfJoints() );
    SetToZero(jac_out);
    pjac_solver_->JntToJac(q_in, jac_out, link_name);
    for (int q_idx = 0; q_idx < q.innerSize(); q_idx++) {
        std::map<int, int >::const_iterator map_it = q_idx_q_nr_map_.find(q_idx);
        if (map_it == q_idx_q_nr_map_.end()) {
            std::cout << "ERROR: KinematicModel::getJacobian: wrong q_idx" << std::endl;
            return;
        }
        int q_nr = map_it->second;
        KDL::Twist t = jac_out.getColumn(q_nr);
        for (int d_idx = 0; d_idx < 6; d_idx++) {
            jac(d_idx, q_idx) = t[d_idx];
        }
    }
}

void KinematicModel::getJointValuesKDL(const Eigen::VectorXd &q, KDL::JntArray &q_kdl) const {
    for (int q_nr = 0; q_nr < tree_.getNrOfJoints(); q_nr++) {
//        std::cout<<"KinematicModel::getJointValuesKDL q_nr=" << q_nr << std::endl;
        std::map<int, int >::const_iterator it = q_nr_q_idx_map_.find(q_nr);
        if (it != q_nr_q_idx_map_.end()) {
            int q_idx = it->second;
            if (q_idx >= q.innerSize()) {
                std::cout<< "ERROR: q_idx >= q.innerSize()   " << q_idx << " " << q.innerSize() << std::endl;
                *((int*)0) = 0;
            }
            q_kdl(q_nr) = q[q_idx];
        }
        else {
            std::map<int, int >::const_iterator ign_it = ign_q_nr_q_idx_map_.find(q_nr);
            if (ign_it != ign_q_nr_q_idx_map_.end()) {
                int q_idx = ign_it->second;
                if (q_idx >= ign_q_.innerSize()) {
                    std::cout<< "ERROR: q_idx >= q.innerSize()   " << q_idx << " " << ign_q_.innerSize() << std::endl;
                    *((int*)0) = 0;
                }
                q_kdl(q_nr) = ign_q_[q_idx];
//                std::cout << "setting ignored joint " << q_nr << " value " << q_kdl(q_nr) << std::endl;
            }
            else {
                std::cout << "ERROR: KinematicModel::getJointValuesKDL: joint is neither in active nor ignored" << std::endl;
                return;
            }
        }
    }
    // update mimic joints
    for (std::map<int, boost::shared_ptr<Mimic > >::const_iterator j_it = q_nr_joint_mimic_map_.begin(); j_it != q_nr_joint_mimic_map_.end(); j_it++) {
        q_kdl(j_it->first) = q_kdl(j_it->second->q_nr_) * j_it->second->multiplier_ + j_it->second->offset_;

//        std::cout << "setting mimic joint " << j_it->first << " to " << q_kdl(j_it->first) <<
//            " from joint " << j_it->second->q_nr_ << " (" << q_kdl(j_it->second->q_nr_) << ")  mul: " << j_it->second->multiplier_ << " offset: " << j_it->second->offset_ << std::endl;
    }
}

void KinematicModel::calculateFk(KDL::Frame &T, const std::string &link_name, const Eigen::VectorXd &q) const {
    KDL::JntArray q_in( tree_.getNrOfJoints() );
    getJointValuesKDL(q, q_in);
    pfk_solver_->JntToCart(q_in, T, link_name);
}

void KinematicModel::getJacobiansForPairX(Jacobian &jac1, Jacobian &jac2,
                                        const std::string &link_name1, const KDL::Vector &x1,
                                        const std::string &link_name2, const KDL::Vector &x2, const Eigen::VectorXd &q) const {

    std::set<unsigned int> link1_chain;
    KDL::SegmentMap::const_iterator root = tree_.getRootSegment();
    KDL::SegmentMap::const_iterator end = tree_.getSegments().end();

    for (KDL::SegmentMap::const_iterator seg_it = tree_.getSegment(link_name1); seg_it != root && seg_it != end; seg_it = seg_it->second.parent) {
        if (seg_it->second.segment.getJoint().getType() == KDL::Joint::None) {
            continue;
        }
        link1_chain.insert(seg_it->second.q_nr);
    }

    std::string common_link_name;

    for (KDL::SegmentMap::const_iterator seg_it = tree_.getSegment(link_name2); seg_it != root && seg_it != end; seg_it = seg_it->second.parent) {
        if (seg_it->second.segment.getJoint().getType() == KDL::Joint::None) {
            continue;
        }
        if (link1_chain.count(seg_it->second.q_nr) > 0) {
            common_link_name = seg_it->second.segment.getName();
            break;
        }
    }

    KDL::JntArray q_kdl( tree_.getNrOfJoints() );
    getJointValuesKDL(q, q_kdl);

    getJacobianForX(jac1, link_name1, x1, q_kdl, common_link_name);
    getJacobianForX(jac2, link_name2, x2, q_kdl, common_link_name);
}

void KinematicModel::getJacobianForX(Jacobian &jac, const std::string &link_name, const KDL::Vector &x, const KDL::JntArray &q_kdl, const std::string &base_name) const {
        // Lets search the tree-element
        // If segmentname is not inside the tree, back out:
        // Let's make the jacobian zero:
        for (int q_idx = 0; q_idx < ndof_; q_idx++) {
            for (int dof_idx = 0; dof_idx < 6; dof_idx++) {
                jac(dof_idx, q_idx) = 0.0;
            }
        }

        KDL::Frame T_total(x);
        KDL::SegmentMap::const_iterator root;
        if (base_name.empty()) {
            root = tree_.getRootSegment();
        }
        else {
            root = tree_.getSegment(base_name);
        }

        // Lets recursively iterate until we are in the root segment
        KDL::SegmentMap::const_iterator it = tree_.getSegment(link_name);
        if (it == tree_.getSegments().end()) {
            // link_name not found in the kinematic model - return zero jacobian
            return;
        }
        while (it != root) {    //l_index != root_index:
            // get the corresponding q_nr for this TreeElement:
            // get the pose of the segment:

            const KDL::Segment &seg_kdl = it->second.segment;
            int q_idx = -1;
            double q_seg = 0.0;
            bool addToJacobian = false;
            if (seg_kdl.getJoint().getType() == KDL::Joint::None) {
                addToJacobian = false;
                q_idx = -1;
                q_seg = 0.0;
            }
            else {
                q_seg = q_kdl(it->second.q_nr);
                std::map<int, int >::const_iterator map_it = q_nr_q_idx_map_.find(it->second.q_nr);
                if (map_it != q_nr_q_idx_map_.end()) {
                    addToJacobian = true;
                    q_idx = map_it->second;
                }
                else {
                    addToJacobian = false;
                }

            }

            KDL::Frame T_local = seg_kdl.pose(q_seg);       // T_local = T_L(i)_L(i+1)
            // calculate new T_end:
            T_total = T_local * T_total;                    // T_total = T_L(i)_L(e)
            // get the twist of the segment:
            if (addToJacobian) {
                KDL::Twist t_local = seg_kdl.twist(q_seg, 1.0);
                // transform the endpoint of the local twist to the global endpoint:
                t_local = t_local.RefPoint(T_total.p - T_local.p);
                // transform the base of the twist to the endpoint
                t_local = T_total.M.Inverse(t_local);
                // store the twist in the jacobian:
                for (int dof_idx = 0; dof_idx < 6; dof_idx++) {
                    jac(dof_idx, q_idx) = t_local[dof_idx];
                }
            }

            // goto the parent
            it = it->second.parent;
        }

        // change Base
        for (int q_idx = 0; q_idx < ndof_; q_idx++) {
            KDL::Twist t;
            for (int dof_idx = 0; dof_idx < 6; dof_idx++) {
                t[dof_idx] = jac(dof_idx, q_idx);
            }

            t = T_total.M * t;
            for (int dof_idx = 0; dof_idx < 6; dof_idx++) {
                jac(dof_idx, q_idx) = t[dof_idx];
            }
        }
}

double KinematicModel::getLowerLimit(int q_idx) const {
    return joint_lower_limit_q_idx_[q_idx];
}

double KinematicModel::getUpperLimit(int q_idx) const {
    return joint_upper_limit_q_idx_[q_idx];
}

void KinematicModel::setLowerLimit(int q_idx, double limit) {
    joint_lower_limit_q_idx_[q_idx] = limit;
}

void KinematicModel::setUpperLimit(int q_idx, double limit) {
    joint_upper_limit_q_idx_[q_idx] = limit;
}

bool KinematicModel::getJointAxisAndOrigin(int q_idx, KDL::Vector &axis, KDL::Vector &origin) const {
    std::map<int, std::string>::const_iterator it = q_idx_link_name_map_.find(q_idx);
    if (it == q_idx_link_name_map_.end()) {
        return false;
    }

    KDL::SegmentMap::const_iterator seg_it = tree_.getSegment( it->second );
    axis = seg_it->second.segment.getJoint().JointAxis();
    origin = seg_it->second.segment.getJoint().JointOrigin();
    return true;
}

bool KinematicModel::getJointLinkName(int q_idx, std::string &link_name) const {
    std::map<int, std::string>::const_iterator it = q_idx_link_name_map_.find(q_idx);
    if (it == q_idx_link_name_map_.end()) {
        return false;
    }

    link_name = it->second;
    return true;
}

int KinematicModel::getDofCount() const {
    return ndof_;
}

