#include <velocity_scaling_iso15066/microinterpolator.h>


namespace cnr
{

namespace control
{


Microinterpolator::Microinterpolator()
{
  m_order=1;
}

void Microinterpolator::setSplineOrder(const unsigned int& order)
{
  if (m_order<5)
    m_order=order;
  else
    ROS_WARN("Interpolation order should be less or equal to 4");
}

bool Microinterpolator::setTrajectory(const trajectory_msgs::JointTrajectoryPtr& trj)
{
  if (!trj)
  {
    ROS_ERROR("Trajectory is not set");
    return false;
  }
  m_trj=trj;
  return m_trj->points.size()>0;
}

bool Microinterpolator::interpolate(const ros::Duration& time, trajectory_msgs::JointTrajectoryPoint& pnt, const double& scaling)
{
  if (!m_trj)
  {
    ROS_ERROR("Trajectory is not set");
    return false;
  }
  
  if (m_trj->points.size()==0)
  {
    ROS_ERROR("Trajectory is empty");
    return false;
  }

  if ((time-m_trj->points.at(0).time_from_start).toSec()<0)
  {
    pnt=m_trj->points.at(0);
    pnt.effort.resize(m_trj->points.at(0).positions.size(),0);
    ROS_ERROR("Negative time, time=%f, trajectory start time %f",time.toSec(),m_trj->points.at(0).time_from_start.toSec());
    return false;
  }
  
  if ((time-m_trj->points.back().time_from_start).toSec()>=0)
  {
    unsigned int nAx=m_trj->points.back().positions.size();
    pnt=m_trj->points.back();
    for (unsigned int iAx=0;iAx<nAx;iAx++)
    {
      pnt.velocities.at(iAx)=0;
      pnt.accelerations.at(iAx)=0;
    }
    pnt.effort.resize(m_trj->points.back().positions.size(),0);
    return true;
  }
  
  for (unsigned int iPnt=1;iPnt<m_trj->points.size();iPnt++)
  {
    if ( ((time-m_trj->points.at(iPnt).time_from_start).toSec()<0) && ((time-m_trj->points.at(iPnt-1).time_from_start).toSec()>=0) )
    {
      unsigned int nAx=m_trj->points.at(iPnt).positions.size();
      pnt.positions.resize(nAx,0);
      pnt.velocities.resize(nAx,0);
      pnt.accelerations.resize(nAx,0);
      pnt.effort.resize(nAx,0);
      pnt.time_from_start=time;
      double delta_time=std::max(1.0e-6,(m_trj->points.at(iPnt).time_from_start-m_trj->points.at(iPnt-1).time_from_start).toSec());
      double t=(time-m_trj->points.at(iPnt-1).time_from_start).toSec();
      double ratio=t/delta_time;
      for (unsigned int iAx=0;iAx<nAx;iAx++)
      {
        //spline order
        if (m_order==0)
        {
          pnt.positions.at(iAx)=m_trj->points.at(iPnt-1).positions.at(iAx)+ratio*(m_trj->points.at(iPnt).positions.at(iAx)-m_trj->points.at(iPnt-1).positions.at(iAx));
          pnt.velocities.at(iAx)=(m_trj->points.at(iPnt).positions.at(iAx)-m_trj->points.at(iPnt-1).positions.at(iAx))/delta_time;
        }
        else if (m_order==1)
        {
          double& p0_1=m_trj->points.at(iPnt-1).positions.at(iAx);
          double& p0_2=m_trj->points.at(iPnt-1).velocities.at(iAx);
          double& pf_1=m_trj->points.at(iPnt).positions.at(iAx);
          double& pf_2=m_trj->points.at(iPnt).velocities.at(iAx);
          
          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = -1.0/(delta_time*delta_time)*(p0_1*3.0-pf_1*3.0+delta_time*p0_2*2.0+delta_time*pf_2);
          double c4 = 1.0/(delta_time*delta_time*delta_time)*(p0_1*2.0-pf_1*2.0+delta_time*p0_2+delta_time*pf_2);
          
          pnt.positions.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t);
          pnt.velocities.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0;
          pnt.accelerations.at(iAx) = c3*2.0+c4*t*6.0;
        }
        else if (m_order==2)
        {
          double& p0_1=m_trj->points.at(iPnt-1).positions.at(iAx);
          double& p0_2=m_trj->points.at(iPnt-1).velocities.at(iAx);
          double& p0_3=m_trj->points.at(iPnt-1).accelerations.at(iAx);
          double& pf_1=m_trj->points.at(iPnt).positions.at(iAx);
          double& pf_2=m_trj->points.at(iPnt).velocities.at(iAx);
          double& pf_3=m_trj->points.at(iPnt).accelerations.at(iAx);
          
          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3*(1.0/2.0);
          double c4 = 1.0/(delta_time*delta_time*delta_time)*(p0_1*2.0E1-pf_1*2.0E1+delta_time*p0_2*1.2E1+delta_time*pf_2*8.0+(delta_time*delta_time)*p0_3*3.0-(delta_time*delta_time)*pf_3)*(-1.0/2.0);
          double c5 = 1.0/(delta_time*delta_time*delta_time*delta_time)*(p0_1*3.0E1-pf_1*3.0E1+delta_time*p0_2*1.6E1+delta_time*pf_2*1.4E1+(delta_time*delta_time)*p0_3*3.0-(delta_time*delta_time)*pf_3*2.0)*(1.0/2.0);
          double c6 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.2E1-pf_1*1.2E1+delta_time*p0_2*6.0+delta_time*pf_2*6.0+(delta_time*delta_time)*p0_3-(delta_time*delta_time)*pf_3)*(-1.0/2.0);
          
          
          pnt.positions.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t)+c5*(t*t*t*t)+c6*(t*t*t*t*t);
          pnt.velocities.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0+c5*(t*t*t)*4.0+c6*(t*t*t*t)*5.0;
          pnt.accelerations.at(iAx) = c3*2.0+c4*t*6.0+c5*(t*t)*1.2E1+c6*(t*t*t)*2.0E1;
        }
        else if (m_order==3)
        {
          double& p0_1=m_trj->points.at(iPnt-1).positions.at(iAx);
          double& p0_2=m_trj->points.at(iPnt-1).velocities.at(iAx);
          double& p0_3=m_trj->points.at(iPnt-1).accelerations.at(iAx);
          double& pf_1=m_trj->points.at(iPnt).positions.at(iAx);
          double& pf_2=m_trj->points.at(iPnt).velocities.at(iAx);
          double& pf_3=m_trj->points.at(iPnt).accelerations.at(iAx);
          // initial and final jerks set equal to zero

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3*(1.0/2.0);
          double c4 = 0.0;
          double c5 = 1.0/(delta_time*delta_time*delta_time*delta_time)*(p0_1*1.4E1-pf_1*1.4E1+delta_time*p0_2*8.0+delta_time*pf_2*6.0+(delta_time*delta_time)*p0_3*2.0-(delta_time*delta_time)*pf_3)*(-5.0/2.0);
          double c6 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*8.4E1-pf_1*8.4E1+delta_time*p0_2*4.5E1+delta_time*pf_2*3.9E1+(delta_time*delta_time)*p0_3*1.0E1-(delta_time*delta_time)*pf_3*7.0);
          double c7 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.4E2-pf_1*1.4E2+delta_time*p0_2*7.2E1+delta_time*pf_2*6.8E1+(delta_time*delta_time)*p0_3*1.5E1-(delta_time*delta_time)*pf_3*1.3E1)*(-1.0/2.0);
          double c8 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.0E1-pf_1*1.0E1+delta_time*p0_2*5.0+delta_time*pf_2*5.0+(delta_time*delta_time)*p0_3-(delta_time*delta_time)*pf_3)*2.0;
          
          
          pnt.positions.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t)+c5*(t*t*t*t)+c6*(t*t*t*t*t)+c7*(t*t*t*t*t*t)+c8*(t*t*t*t*t*t*t);
          pnt.velocities.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0+c5*(t*t*t)*4.0+c6*(t*t*t*t)*5.0+c7*(t*t*t*t*t)*6.0+c8*(t*t*t*t*t*t)*7.0;
          pnt.accelerations.at(iAx) = c3*2.0+c4*t*6.0+c5*(t*t)*1.2E1+c6*(t*t*t)*2.0E1+c7*(t*t*t*t)*3.0E1+c8*(t*t*t*t*t)*4.2E1;
        }
        else if (m_order==4)
        {
          double& p0_1=m_trj->points.at(iPnt-1).positions.at(iAx);
          double& p0_2=m_trj->points.at(iPnt-1).velocities.at(iAx);
          double& p0_3=m_trj->points.at(iPnt-1).accelerations.at(iAx);
          double& pf_1=m_trj->points.at(iPnt).positions.at(iAx);
          double& pf_2=m_trj->points.at(iPnt).velocities.at(iAx);
          double& pf_3=m_trj->points.at(iPnt).accelerations.at(iAx);
          // initial and final jerks and snaps set equal to zero

          double c1 = p0_1;
          double c2 = p0_2;
          double c3 = p0_3*(1.0/2.0);
          double c4 = 0.0;
          double c5 = 0.0;
          double c6 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*3.6E1-pf_1*3.6E1+delta_time*p0_2*2.0E1+delta_time*pf_2*1.6E1+(delta_time*delta_time)*p0_3*5.0-(delta_time*delta_time)*pf_3*3.0)*(-7.0/2.0);
          double c7 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.2E2-pf_1*1.2E2+delta_time*p0_2*6.4E1+delta_time*pf_2*5.6E1+(delta_time*delta_time)*p0_3*1.5E1-(delta_time*delta_time)*pf_3*1.1E1)*(7.0/2.0);
          double c8 = -1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*5.4E2-pf_1*5.4E2+delta_time*p0_2*2.8E2+delta_time*pf_2*2.6E2+(delta_time*delta_time)*p0_3*6.3E1-(delta_time*delta_time)*pf_3*5.3E1);
          double c9 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*1.26E2-pf_1*1.26E2+delta_time*p0_2*6.4E1+delta_time*pf_2*6.2E1+(delta_time*delta_time)*p0_3*1.4E1-(delta_time*delta_time)*pf_3*1.3E1)*(5.0/2.0);
          double c10 = 1.0/(delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time*delta_time)*(p0_1*2.8E1-pf_1*2.8E1+delta_time*p0_2*1.4E1+delta_time*pf_2*1.4E1+(delta_time*delta_time)*p0_3*3.0-(delta_time*delta_time)*pf_3*3.0)*(-5.0/2.0);
          
          
          pnt.positions.at(iAx)     = c1+c2*t+c3*(t*t)+c4*(t*t*t)+c5*(t*t*t*t)+c6*(t*t*t*t*t)+c7*(t*t*t*t*t*t)+c8*(t*t*t*t*t*t*t)+c9*(t*t*t*t*t*t*t*t)+c10*(t*t*t*t*t*t*t*t*t);
          pnt.velocities.at(iAx)    = c2+c3*t*2.0+c4*(t*t)*3.0+c5*(t*t*t)*4.0+c6*(t*t*t*t)*5.0+c7*(t*t*t*t*t)*6.0+c8*(t*t*t*t*t*t)*7.0+c9*(t*t*t*t*t*t*t)*8.0+c10*(t*t*t*t*t*t*t*t)*9.0;
          pnt.accelerations.at(iAx) = c3*2.0+c4*t*6.0+c5*(t*t)*1.2E1+c6*(t*t*t)*2.0E1+c7*(t*t*t*t)*3.0E1+c8*(t*t*t*t*t)*4.2E1+c9*(t*t*t*t*t*t)*5.6E1+c10*(t*t*t*t*t*t*t)*7.2E1;
        }
        
        pnt.velocities.at(iAx)    *= scaling ;
        pnt.accelerations.at(iAx) *= scaling*scaling;
      }
      
      
      break;
    }
  }
  
  
  return true;
}

ros::Duration Microinterpolator::trjTime()
{
  if (m_trj)
    return m_trj->points.back().time_from_start;
  else
    return ros::Duration(0);
}


}  // namespace control
}  // namespace cnr
