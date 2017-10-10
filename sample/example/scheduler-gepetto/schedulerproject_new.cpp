#include <schedulerproject.hh>

using namespace OpenHRP;
using namespace std;
using namespace hrpGep;

#if 0
#define ODEBUG(X)
#else
#define ODEBUG(X) X
#endif

#define ODEBUG3(X)

template <typename X, typename X_ptr>
X_ptr checkCorbaServer(std::string n, CosNaming::NamingContext_var &cxt)
{
  CosNaming::Name ncName;
  ncName.length(1);
  ncName[0].id = CORBA::string_dup(n.c_str());
  ncName[0].kind = CORBA::string_dup("");
  X_ptr srv = NULL;
  try {
    srv = X::_narrow(cxt->resolve(ncName));
  } catch(const CosNaming::NamingContext::NotFound &exc) {
    std::cerr << n << " not found: ";
    switch(exc.why) {
    case CosNaming::NamingContext::missing_node:
      std::cerr << "Missing Node" << std::endl;
    case CosNaming::NamingContext::not_context:
      std::cerr << "Not Context" << std::endl;
      break;
    case CosNaming::NamingContext::not_object:
      std::cerr << "Not Object" << std::endl;
      break;
    }
    return (X_ptr)NULL;
  } catch(CosNaming::NamingContext::CannotProceed &exc) {
    std::cerr << "Resolve " << n << " CannotProceed" << std::endl;
  } catch(CosNaming::NamingContext::AlreadyBound &exc) {
    std::cerr << "Resolve " << n << " InvalidName" << std::endl;
  }
  return srv;
}


SchedulerProject::SchedulerProject()
{
  simulationData_ = 
    std::make_shared<SimulationItem>();
  aListOfModelItems_ = 
    std::make_shared< std::vector<ModelItem> >();
  aListOfCollisionPairItems_ = 
    std::make_shared< std::vector<CollisionPairItem> > ();

}

SchedulerProject::~SchedulerProject()
{
}

void SchedulerProject::parseOptions(int argc, char *argv[])
{
  std::string ProjectFname;
  
  for (int i=1; i<argc; i++)
    {
      if (strcmp("-ORBconfig", argv[i])==0 || strcmp("-ORBInitRef", argv[i])==0 )
	{
	  ++i;	// skip ORB parameter
	}
      else if (strcmp("-url", argv[i])==0)
	{
	  ProjectFname = argv[++i];
	}
      else if (strcmp("-timeStep", argv[i])==0)
	{
	  simulationData_.get()->timeStep = atof(argv[++i]);
	}
      
    }
  
  
  projectfilename_ = "file://" + ProjectFname;
  ODEBUG(std::cout << "Model: " << projectfilename_ << std::endl);
}

void SchedulerProject::loadModels(int argc,char * argv[])
{
  // Create the list of logs.
  listOfLogs_.resize(aListOfModelItems_.get()->size());

  for(unsigned int i=0;i<aListOfModelItems_.get()->size();i++)
    {
      // Create a body info structure.
      (*aListOfModelItems_.get())[i].bodyInfoVar = 
	hrp::loadBodyInfo((*aListOfModelItems_.get())[i].url.c_str(), 
			  argc, argv);
      if (!(*aListOfModelItems_.get())[i].bodyInfoVar )
	{
	  cerr << "ModelLoader: " 
	       << (*aListOfModelItems_.get())[i].url
	       << " cannot be loaded" << endl;
	  return;
	  
	}
      // Create a log object.
      std::string directory("/tmp/");
      listOfLogs_[i].setNameAndPath((*aListOfModelItems_.get())[i].name,
				     directory);
     }
 }

 void SchedulerProject::initCorba(int argc, char * argv[])
 {
   // initialize CORBA 
   CORBA::ORB_var orb;
   orb = CORBA::ORB_init(argc, argv);

   // ROOT POA
   CORBA::Object_var poaObj = orb -> resolve_initial_references("RootPOA");
   PortableServer::POA_var rootPOA = PortableServer::POA::_narrow(poaObj);

   // get reference to POA manager
   PortableServer::POAManager_var manager = rootPOA -> the_POAManager();


   {
     CORBA::Object_var	nS = orb->resolve_initial_references("NameService");
     cxt_ = CosNaming::NamingContext::_narrow(nS);
   }
 }

 void SchedulerProject::initOLV(int argc, char * argv[])
 {
   
   olv_ = hrp::getOnlineViewer(argc, argv);

   bool ok =false;
   unsigned int Counter=0;
   do
     {
       if (CORBA::is_nil( olv_ )) 
	 {
	   std::cerr << "OnlineViewer not found" << std::endl;
	   return;
	 }
       try 
	 {
	   for(unsigned int i=0;i<aListOfModelItems_.get()->size();i++)
	     {
	       cerr << (*aListOfModelItems_.get())[i].name.c_str()<< " " 
		    << (*aListOfModelItems_.get())[i].url.c_str() << std::endl;

	       olv_->load((*aListOfModelItems_.get())[i].name.c_str(),
			  (*aListOfModelItems_.get())[i].url.c_str());
	     }
	   
	   olv_->clearLog();
	   ok=true;
	 } 
       catch (CORBA::SystemException& ex) 
	 {
	   cerr << "Failed to connect GrxUI. Attempt: " 
		<< Counter << " "
		<< ex._name() << " " << ex._rep_id()
		<< endl;
	 }
       // Wait 100 ms
       usleep(100000);
       Counter++;
     }
   // Try during 20s.
   while(Counter<200 && !ok);
   
 }


 void SchedulerProject::setBodyAngle(JointData &aJointData,
				     //OpenHRP::BodyInfo_var  aBodyInfo,
				     std::string &CharacterName,
				     CORBA::String_var CORBAbodyName)
 {
   // Set angle value.
   DblSequence corba_angle;
   corba_angle.length(1);
   corba_angle[0] = aJointData.angle;
   ODEBUG(std::cout << "setBodyAngle: " << CharacterName << " " << CORBAbodyName << " " << aJointData.angle<<std::endl);
   dynamicsSimulator_->setCharacterLinkData( CharacterName.c_str(),
					     CORBAbodyName,
					     DynamicsSimulator::JOINT_VALUE, 
					     corba_angle );
  }

  void SchedulerProject::setBodyVelocity(JointData &aJointData,
					 string & CharacterName,
					 CORBA::String_var CORBAbodyName)
  {
    DblSequence   velocity; velocity.length(6);
    if ((aJointData.velocity.size()>0) && 
	(aJointData.angularVelocity.size()>0))
      {
	for(unsigned int idVel=0;idVel<3;idVel++)
	  { velocity[idVel] = aJointData.velocity[idVel];
	    velocity[idVel+3] = aJointData.angularVelocity[idVel];
	  }
      }
    else
      {
	for(unsigned int idVel=0;idVel<3;idVel++)
	  { velocity[idVel] =  velocity[idVel+3] = 0.0;
	  }
      }
    dynamicsSimulator_->setCharacterLinkData(CharacterName.c_str(),
					     CORBAbodyName,
					     DynamicsSimulator::ABS_VELOCITY, 
					     velocity );
 }

 void SchedulerProject::setBodyAcceleration(string & CharacterName,
					 CORBA::String_var CORBAbodyName)
 {
   DblSequence acceleration; acceleration.length(6);
   for(unsigned int i=0;i<6;i++)
     acceleration[i] = 0.0;
   dynamicsSimulator_->setCharacterLinkData(CharacterName.c_str(),
					    CORBAbodyName,
					    DynamicsSimulator::ABS_ACCELERATION, 
					    acceleration );
 }
  
 void SchedulerProject::setBodyTorque(string & CharacterName,
					 CORBA::String_var CORBAbodyName)
 {
   ODEBUG(std::cout << "setBodyTorque: " << CharacterName << " : "
	  << CORBAbodyName << std::endl);
   DblSequence torque; torque.length(1);
   torque[0] = 0.0;
   dynamicsSimulator_->setCharacterLinkData(CharacterName.c_str(),
					    CORBAbodyName,
					    DynamicsSimulator::JOINT_TORQUE, 
					    torque );
 }

 
 struct AxisAngle4f
 {
   double x,y,z,angle;
 };

 struct rotationMatrix3d
 {
   double m00, m01, m02, m10, m11, m12, m20, m21, m22;
   constexpr static double EPS = 1.110223024E-16;

   void fromAxisAngle4(AxisAngle4f &a1)
   {
     double mag = sqrt( a1.x*a1.x + a1.y*a1.y + a1.z*a1.z);

     if( mag < EPS ) {
       m00 = 1.0;
       m01 = 0.0;
       m02 = 0.0;

       m10 = 0.0;
       m11 = 1.0;
       m12 = 0.0;

       m20 = 0.0;
       m21 = 0.0;
       m22 = 1.0;
     } else {
       mag = 1.0/mag;
       double ax = a1.x*mag;
       double ay = a1.y*mag;
       double az = a1.z*mag;

       double sinTheta = sin(a1.angle);
       double cosTheta = cos(a1.angle);
       double t = 1.0 - cosTheta;

       double xz = ax * az;
       double xy = ax * ay;
       double yz = ay * az;

       m00 = t * ax * ax + cosTheta;
       m01 = t * xy - sinTheta * az;
       m02 = t * xz + sinTheta * ay;

       m10 = t * xy + sinTheta * az;
       m11 = t * ay * ay + cosTheta;
       m12 = t * yz - sinTheta * ax;

       m20 = t * xz - sinTheta * ay;
       m21 = t * yz + sinTheta * ax;
       m22 = t * az * az + cosTheta;
     }
   }
 };

void SchedulerProject::setBodyAbsolutePosition(JointData &aJointData,
					       string &CharacterName,
					       CORBA::String_var CORBAbodyName)
{
  ODEBUG3(std::cout << "Going through setBodyAbsolute Position for :" 
	  << CharacterName << " " 
	  << CORBAbodyName << " " 
	  << aJointData.translation.size() << " "
	  << aJointData.rotation.size()
	  << std::endl);
  DblSequence TransformArray; TransformArray.length(12);

  if ((aJointData.translation.size()>0) && 
      (aJointData.rotation.size()>0))
    {
      ODEBUG(std::cout << "initialize absolute position of :"
	     << CharacterName << " link: " << CORBAbodyName << std::endl);
	     
      for(unsigned int i=0;i<3;i++)
	TransformArray[i] = aJointData.translation[i];
      
      AxisAngle4f anAxisAngle;
      anAxisAngle.x = aJointData.rotation[0];
      anAxisAngle.y = aJointData.rotation[1];
      anAxisAngle.z = aJointData.rotation[2];
      anAxisAngle.angle = aJointData.rotation[3];
      rotationMatrix3d aRotation;
      aRotation.fromAxisAngle4(anAxisAngle);
      TransformArray[3] = aRotation.m00;TransformArray[4]  = aRotation.m01;TransformArray[5]  = aRotation.m02;
      TransformArray[6] = aRotation.m10;TransformArray[7]  = aRotation.m11;TransformArray[8]  = aRotation.m12;
      TransformArray[9] = aRotation.m20;TransformArray[10] = aRotation.m21;TransformArray[11] = aRotation.m22;
    }
  else
    {
      for(unsigned int i=0;i<3;i++)
	TransformArray[i] = 0.0;
      TransformArray[3] = 1.0;TransformArray[4]  = 0.0;TransformArray[5]  = 0.0;
      TransformArray[6] = 0.0;TransformArray[7]  = 1.0;TransformArray[8]  = 0.0;
      TransformArray[9] = 0.0;TransformArray[10] = 0.0;TransformArray[11] = 1.0;
    
    }
  dynamicsSimulator_->setCharacterLinkData(	CharacterName.c_str(),
						CORBAbodyName,
						DynamicsSimulator::ABS_TRANSFORM, 
						TransformArray );
  ODEBUG(for(unsigned int i=0;i<4;i++) 
	   {
	     for(unsigned int j=0;j<3;j++)
	       std::cout << TransformArray[i*3+j] << " ";
	     std::cout << std::endl;
	   }
	     
	 );
}

void SchedulerProject::setBodyMode(JointData &aJointData,
				   //OpenHRP::BodyInfo_var  aBodyInfo,
				   string &CharacterName,
				    CORBA::String_var CORBAbodyName)
{
  DblSequence wdata;
  wdata.length(1);
  ODEBUG(std::cout << "In " << CharacterName << "setBodyMode for " << CORBAbodyName);
  if (aJointData.mode==0)
    {
      wdata[0] = 1.0;
      dynamicsSimulator_->setCharacterLinkData(	CharacterName.c_str(),
						CORBAbodyName, 
						DynamicsSimulator::POSITION_GIVEN, 
						wdata ); 
      ODEBUG(std::cout << ": High Gain ");
    }
  else 
    {
      wdata[0] = -1.0;
      dynamicsSimulator_->setCharacterLinkData(	CharacterName.c_str(),
						CORBAbodyName, 
						DynamicsSimulator::POSITION_GIVEN, 
						wdata ); 
      ODEBUG(std::cout << ": Torque ");
    }
  ODEBUG(std::cout << std::endl);
}

void SchedulerProject::initRobotsFrictionData()
{
  ODEBUG(std::cout << "initRobotsFrictionData() start" << std::endl);
  for(unsigned int idModel=0;idModel<aListOfModelItems_.get()->size();idModel++) {
    // If not a robot, let it go
    if(!(*aListOfModelItems_.get())[idModel].isRobot);
    else {
      std::string & CharacterName = (*aListOfModelItems_.get())[idModel].name;
      std::string &frictionFileName = (*aListOfModelItems_.get())[idModel].jointFrictionFile;        
      ODEBUG(std::cout << (*aListOfModelItems_.get())[idModel].name << " " 
             << (*aListOfModelItems_.get())[idModel].jointFrictionFile<< std::endl );
      //If no file, do nothing. Default friction coeff=0.0;
      if (frictionFileName.length()==0);
      else {
        //Read gains and names
        std::list<std::string> jNameList;
        std::list<int> Kv_pList, Kv_nList,Ka_pList, Ka_nList,Kf_pList, Kf_nList;
        std::ifstream ifs;
        ifs.open(frictionFileName.c_str());
        if (!ifs.is_open())
          return;
        while(true) {
          std::string jName_in;
          int Kv_p_in, Kv_n_in,Ka_p_in, Ka_n_in,Kf_p_in, Kf_n_in;
          ifs >> jName_in;
          ifs >> Kv_p_in;          ifs >> Kv_n_in;
          ifs >> Ka_p_in;          ifs >> Ka_n_in;
          ifs >> Kf_p_in;          ifs >> Kf_n_in;
          jNameList.push_back(jName_in);
          Kv_pList.push_back(Kv_p_in);           Kv_nList.push_back(Kv_n_in); 
          Ka_pList.push_back(Ka_p_in);           Ka_nList.push_back(Ka_n_in); 
          Kf_pList.push_back(Kf_p_in);           Kf_nList.push_back(Kf_n_in); 
          if( ifs.eof() ) break;
        }
        ifs.close();

        std::list<int>::const_iterator Kv_p_ = Kv_pList.begin();
        std::list<int>::const_iterator Kv_n_ = Kv_nList.begin();
        std::list<int>::const_iterator Ka_p_ = Ka_pList.begin();
        std::list<int>::const_iterator Ka_n_ = Ka_nList.begin();
        std::list<int>::const_iterator Kf_p_ = Kf_pList.begin();
        std::list<int>::const_iterator Kf_n_ = Kf_nList.begin();

        for (std::list<std::string>::const_iterator jName_ = jNameList.begin();
             jName_ != jNameList.end(); ++jName_) {
          CORBA::String_var CORBAbodyName((*jName_).c_str());
          //set gains and names
          DblSequence friction_gains;
          friction_gains.length(6);
          friction_gains[0] = *Kv_p_;          friction_gains[1] = *Kv_n_;
          friction_gains[2] = *Ka_p_;          friction_gains[3] = *Ka_n_;
          friction_gains[4] = *Kf_p_;          friction_gains[5] = *Kf_n_;
          dynamicsSimulator_->setCharacterLinkData( CharacterName.c_str(),
                                                    CORBAbodyName,
                                                    DynamicsSimulator::JOINT_FRICTION_GAINS, 
                                                    friction_gains);
          ++Kv_p_; ++Kv_n_;
          ++Ka_p_; ++Ka_n_;
          ++Kf_p_; ++Kf_n_;
        }
      }
    }
  }
}

void SchedulerProject::initRobotsJointData()
{
  ODEBUG(std::cout << "initRobotsJointData() *************** " << simulationData_.get()->integrate);
  if (simulationData_.get()->integrate)
    {
      
      // Iterate over the robots.
      for(unsigned int idModel=0;
	  idModel < aListOfModelItems_.get()->size();
	  idModel++)
	{
	  OpenHRP::BodyInfo_var aBodyInfo = 
	    (*aListOfModelItems_.get())[idModel].bodyInfoVar;
	  LinkInfoSequence_var aLinkInfoSequence = aBodyInfo->links();


	  // Iterave over the links
	  for(unsigned int idLinks=0;
	      idLinks<aLinkInfoSequence->length();
	      idLinks++)
	    {
	      CORBA::String_var CORBAbodyName = 
		aLinkInfoSequence[idLinks].name;
	      std::string bodyName(CORBAbodyName);
	      
	      JointData aJointData = 
		(*aListOfModelItems_.get())[idModel].jointsMap[bodyName];
	      std::string & CharacterName = (*aListOfModelItems_.get())[idModel].name;
	      setBodyAbsolutePosition(aJointData,CharacterName,CORBAbodyName);  // Set Body absoluteposition.
	      setBodyAngle(aJointData,CharacterName,CORBAbodyName);     // Set Body angle value.
	      setBodyVelocity(aJointData,CharacterName,CORBAbodyName);  // Set Joint speed.
	      setBodyAcceleration(CharacterName,CORBAbodyName);     // Set Body acceleration.
	      setBodyTorque(CharacterName,CORBAbodyName);           // Set Body torque.
	      setBodyMode(aJointData,CharacterName,CORBAbodyName);  // Set Joint mode.

	    }
	}
    }
    
}

void SchedulerProject::setTorqueToZero()
{
  ODEBUG(std::cout << "setTorqueToZero() begin" << std::endl);
  if (simulationData_.get()->integrate)
    {
      
      // Iterate over the robots.
      for(unsigned int idModel=0;
	  idModel < aListOfModelItems_.get()->size();
	  idModel++)
	{
	  OpenHRP::BodyInfo_var aBodyInfo = 
	    (*aListOfModelItems_.get())[idModel].bodyInfoVar;
	  LinkInfoSequence_var aLinkInfoSequence = aBodyInfo->links();


	  // Iterave over the links
	  for(unsigned int idLinks=0;
	      idLinks<aLinkInfoSequence->length();
	      idLinks++)
	    {
	      CORBA::String_var CORBAbodyName = 
		aLinkInfoSequence[idLinks].name;
	      std::string bodyName(CORBAbodyName);
	      
	      JointData aJointData = 
		(*aListOfModelItems_.get())[idModel].jointsMap[bodyName];
	      std::string & CharacterName = (*aListOfModelItems_.get())[idModel].name;
	      setBodyTorque(CharacterName,CORBAbodyName);           // Set Body torque.
	    }
	}
    }
  ODEBUG(std::cout << "setTorqueToZero() end" << std::endl);    
}

void SchedulerProject::initParallelMecanisms()
{
  ODEBUG(std::cout << "initParallelMecanisms start" << std::endl;)
  // Iterate over the robots.
  for(unsigned int idModel=0;
      idModel < aListOfModelItems_.get()->size();
      idModel++)
    {
      OpenHRP::BodyInfo_var aBodyInfo = 
   	(*aListOfModelItems_.get())[idModel].bodyInfoVar;
      ODEBUG(std::cout << aBodyInfo->name() << std::endl);

      OpenHRP::ExtraJointInfoSequence_var 
	anExtraJointInfoSequence = aBodyInfo->extraJoints();

      ODEBUG(std::cout << "Nb of extra joints:" << anExtraJointInfoSequence->length() << std::endl);
      for(unsigned int idExtraJointInfo=0;
	  idExtraJointInfo<anExtraJointInfoSequence->length();
	  idExtraJointInfo++)
	{
	  ODEBUG(std::cout << "Extra joint "<< anExtraJointInfoSequence[idExtraJointInfo].name <<std::endl);
	  ODEBUG(std::cout << aBodyInfo->name() << " "<<  anExtraJointInfoSequence[idExtraJointInfo].link[0] <<std::endl);
	  ODEBUG(std::cout << aBodyInfo->name() << " "<<  anExtraJointInfoSequence[idExtraJointInfo].link[1] <<std::endl);
	 
	  OpenHRP::DblSequence3 point1,point2,laxis;
	  point1.length(3);point2.length(3); laxis.length(3);
	  for(unsigned int i=0;i<3;i++)
	    {
	      point1[i] = anExtraJointInfoSequence[idExtraJointInfo].point[0][i];
	      point2[i] = anExtraJointInfoSequence[idExtraJointInfo].point[1][i];
	      laxis[i] = anExtraJointInfoSequence[idExtraJointInfo].axis[i];
	    };
	  ODEBUG(std::cout << "(" << point1[0] << "," << point1[1] << ","<< point1[2] << ")" << std::endl);
	  ODEBUG(std::cout << "(" << point2[0] << "," << point2[1] << ","<< point2[2] << ")" << std::endl);
	  ODEBUG(std::cout << anExtraJointInfoSequence[idExtraJointInfo].jointType <<std::endl);
	  ODEBUG(std::cout << "(" << laxis[0] << "," << laxis[1] << ","<< laxis[2] << ")" << std::endl);
	  
	  dynamicsSimulator_->registerExtraJoint(aBodyInfo->name(), anExtraJointInfoSequence[idExtraJointInfo].link[0],
						 aBodyInfo->name(), anExtraJointInfoSequence[idExtraJointInfo].link[1],
   						 point1, 
   						 point2, 
						 anExtraJointInfoSequence[idExtraJointInfo].jointType,
   						 laxis,
   						 anExtraJointInfoSequence[idExtraJointInfo].name);
	}
    }
  ODEBUG(std::cout << "initParallelMecanisms end" << std::endl;)
}

void SchedulerProject::initCollisions()
{
  ODEBUG(std::cout << "Start initCollisions" << std::endl);
  // Iterate over all the collision pair.
  for(unsigned int idCollision=0;
      idCollision<aListOfCollisionPairItems_.get()->size();
      idCollision++)
    {
      CollisionPairItem & aCollisionPI = 
	(*aListOfCollisionPairItems_.get())[idCollision];
      // static/slip friction coefficient 
      double statFric,slipFric;
      statFric = aCollisionPI.staticFriction;
      slipFric = aCollisionPI.slidingFriction;
      double culling_thresh = aCollisionPI.cullingThresh;
      ODEBUG(std::cout << "staticFriction: "<< statFric 
	     << " slipFriction: "<< slipFric 
	     << " cullingThres: "<< culling_thresh
	     << std::endl);
      // Take into accound spring damper if specified
      // usually not a good idea.
      DblSequence6 K, C;    
      if (aCollisionPI.springDamperModel)
	{
	  K.length((CORBA::ULong)aCollisionPI.springConstant.size());
	  C.length((CORBA::ULong)aCollisionPI.damperConstant.size());
	  for(unsigned int i=0;i<aCollisionPI.springConstant.size();i++)
	    { K[i] = aCollisionPI.springConstant[i];
	      ODEBUG(std::cout << "K["<<i<<"]="<<K[i] << std::endl);
	    }
	  for(unsigned int i=0;i<aCollisionPI.damperConstant.size();i++)
	    { C[i] = aCollisionPI.damperConstant[i];
	      ODEBUG(std::cout << "C["<<i<<"]="<<C[i] << std::endl);
	    }
	}
      else 
	{
	  K.length(0); C.length(0);
	}
      ODEBUG(std::cout << "K.length()=" << K.length() << " " 
	     << "C.lenght()=" << C.length() << std::endl);
      ODEBUG(std::cout << "Register collision pair:" 
	     << aCollisionPI.objectName1.c_str() << "|"
	     << aCollisionPI.jointName1.c_str() << "|" 
	     << aCollisionPI.objectName2.c_str() << "|"
	     << aCollisionPI.jointName2.c_str() << "|"
	     << std::endl);
      dynamicsSimulator_->
	registerCollisionCheckPair(aCollisionPI.objectName1.c_str(),
				   aCollisionPI.jointName1.c_str(),
				   aCollisionPI.objectName2.c_str(),
				   aCollisionPI.jointName2.c_str(),
				   statFric,
				   slipFric,
				   K,C,culling_thresh,0.0);
      
	
    }
  ODEBUG(std::cout << "End of initCollisions" << std::endl);
  
}
void SchedulerProject::initDynamicsSimulator()
{
  DynamicsSimulatorFactory_var dynamicsSimulatorFactory;
  dynamicsSimulatorFactory =
    checkCorbaServer <DynamicsSimulatorFactory, DynamicsSimulatorFactory_var> ("DynamicsSimulatorFactory", cxt_);
  
  if (CORBA::is_nil(dynamicsSimulatorFactory)) {
    std::cerr << "DynamicsSimulatorFactory not found" << std::endl;
  }
  
  // Create dynamics simulator
  dynamicsSimulator_ = dynamicsSimulatorFactory->create();

  try
    {
      // Register robots.
      cout << "** Dynamics server setup ** " << endl;
      for(unsigned int i=0;i<aListOfModelItems_.get()->size();i++)
	{
	  OpenHRP::BodyInfo_var aBodyInfo = (*aListOfModelItems_.get())[i].bodyInfoVar;
	  
	  cout << "Character  : |" << (*aListOfModelItems_.get())[i].name << "|" << std::endl
	       << "Model      : |" << aBodyInfo->name() << "|" << std::endl;
	  dynamicsSimulator_->registerCharacter((*aListOfModelItems_.get())[i].name.c_str(), 
					    aBodyInfo);			    
	}
      
      // Enable sensor and gravity.
      CORBA::Double ltimestep = simulationData_.get()->timeStep;
      dynamicsSimulator_->init(ltimestep, 
			       simulationData_.get()->method,
			       DynamicsSimulator::ENABLE_SENSOR);
      DblSequence3 g;
      g.length(3);
      g[0] = 0.0;
      g[1] = 0.0;
      double world_gravity = simulationData_.get()->gravity;  
      // default gravity acceleration [m/s^2]
      g[2] = world_gravity;
      dynamicsSimulator_->setGVector(g);
      
      //initRobotsPose();
      initRobotsJointData();
      initRobotsFrictionData();
      dynamicsSimulator_->calcWorldForwardKinematics();
      
      initCollisions();
      initParallelMecanisms();
      dynamicsSimulator_->initSimulation();
    }
  catch (CORBA::SystemException& ex) 
    {
      cerr << "Failed to connect to dynamicsSimulator: " 
	   << ex._name() << " " << ex._rep_id()
	   << endl;
    }


}

void SchedulerProject::initController()
{
  // Allocate the number of possible controllers.
  aListOfControllers_.resize(aListOfModelItems_.get()->size());
  ODEBUG(std::cout << "initController() start" << std::endl);
  for(unsigned int i=0;i<aListOfModelItems_.get()->size();i++)
    {
      
      std::string &lControllerName =  (*aListOfModelItems_.get())[i].controllerName;
      bool lcontrollerOpenHRP;

      ODEBUG(std::cout << (*aListOfModelItems_.get())[i].name << " " 
	     << (*aListOfModelItems_.get())[i].controllerName<< std::endl );
      // If no controller is specified
      if (lControllerName.length()==0)
	{
	  aListOfControllers_[i].controller_found = false;
	  continue; // Go to the next candidate
	}

      aListOfControllers_[i].controller_found = false;
      unsigned int Counter=0;
      do
	{
	  
	  try 
	    {
	      aListOfControllers_[i].controllerName = lControllerName;
	      aListOfControllers_[i].controller = 
		checkCorbaServer <Controller, Controller_var> 
		(lControllerName.c_str(), cxt_);
	      aListOfControllers_[i].controller_found = true;
	    }
	  catch (CORBA::SystemException& ex) 
	    {
	      cerr << "Failed to connect to " << lControllerName.c_str() << endl;
	    }
	  Counter++;
	}
      while(Counter<200 && !aListOfControllers_[i].controller_found);
      
      
      ODEBUG(std::cout << "initController() connected to " 
	     << lControllerName.c_str() 
	     << std::endl);

      if (CORBA::is_nil(aListOfControllers_[i].controller)) {
	std::cerr << "Controller " << lControllerName << " not found" << std::endl;
	aListOfControllers_[i].controller_found = false;
      }

      ODEBUG(std::cout << "Before initializing controller" << lControllerName 
	     << std::endl);

      if (aListOfControllers_[i].controller_found)
	{
	  ODEBUG(std::cout << lControllerName.c_str() 
		 << " character/model set " 
		 << (*aListOfModelItems_.get())[i].name.c_str()
		 << std::endl);

	  // Set the character/model name used by the controller.
	  aListOfControllers_[i].controller->
	    setModelName(CORBA::string_dup((*aListOfModelItems_.get())[i].name.c_str()));
	  ODEBUG(std::cout << lControllerName.c_str() 
		 << " character/model set " 
		 << std::endl);
		
	  // Set the dynamics simulator
	  aListOfControllers_[i].controller->
	    setDynamicsSimulator(dynamicsSimulator_);

	  ODEBUG(std::cout << lControllerName.c_str() 
		 << " set dynamics simulator for the controller " 
		 << std::endl);

	  // Initialize the controller
	  aListOfControllers_[i].controller->initialize();

	  ODEBUG(std::cout << lControllerName.c_str() 
		 << " initialize the controller " 
		 << std::endl);

	  // Specify the time
	  aListOfControllers_[i].controller->setTimeStep(controlTimeStep_);
	  // State the controller
	  aListOfControllers_[i].controller->start();
	}
    }
  ODEBUG(std::cout << "initController() end" << std::endl);
}

void SchedulerProject::init(int argc,char * argv[])
{
  
  // Parse options
  parseOptions(argc,argv);

  // ================== Project Load ===============================
  aProjectLoader_.loadProject(projectfilename_,
			     simulationData_,
			     aListOfModelItems_,
			     aListOfCollisionPairItems_);

  //================== Model Load ===============================
  loadModels(argc,argv);
  
  //================== CORBA init ===============================
  initCorba(argc,argv);
  
  sleep(5);
  //================= DynamicsSimulator setup ======================
  initDynamicsSimulator();
  // ==================  Controller setup ==========================
  initController();
  //==================== OnlineViewer (GrxUI) setup ===============
  initOLV(argc,argv);

}

void SchedulerProject::mainLoop()
{
  // ==================  log file   ======================
  static ofstream log_file;
  log_file.open("samplePD.log");
  
  WorldState_var state;
  int i=0;
  int j = 0;
  double time=0.0;
  double controlTime=0.0;

  while ( 1 ) {
    bool control=false;
    if(controlTime <= time){
      control=true;
      j++;
    }

    if(control)
      {
	for(unsigned int i=0;i<aListOfControllers_.size();i++)
	  {
	    if (aListOfControllers_[i].controller_found)
	      aListOfControllers_[i].controller->input();
	  }
      }

    i++;
    if (i%1000==0)
      std::cout << " Counter: " << i << std::endl;

    time = simulationData_.get()->timeStep * i;
    controlTime = controlTimeStep_ * j;

    if(control)
      {
	for(unsigned int i=0;i<aListOfControllers_.size();i++)
	  {
	    if (aListOfControllers_[i].controller_found)
	      aListOfControllers_[i].controller->control();
	  }
      }
      
    // ================== simulate one step ==============
    dynamicsSimulator_->stepSimulation();					
               
    // ================== viewer update ====================
    try {
      dynamicsSimulator_ -> getWorldState( state );
      olv_->update( state );
    } catch (CORBA::SystemException& ex) {
      return ;
    }

    // ================== call control =====================
    if(control)
      {
	for(unsigned int i=0;i<aListOfControllers_.size();i++)
	  {
	    if (aListOfControllers_[i].controller_found)
	      aListOfControllers_[i].controller->output();
	  }
	if (aListOfControllers_.size()==0)
	  {
	    setTorqueToZero();
	  }
      }
    // ================== save log =====================
    saveLog(time);

    if( time > simulationData_.get()->totalTime ) break;

    //usleep(5000);
  }

  //controller->stop();
  log_file.close();
  dynamicsSimulator_->destroy();

}

void SchedulerProject::saveLog(double ltime)
{
  // Iterate over all the characters loaded 
  for(unsigned int idModel=0;
      idModel<listOfLogs_.size();
      idModel++)
    {
      OpenHRP::SensorState_var sstate;
      OpenHRP::BodyInfo_var aBodyInfo = 
	(*aListOfModelItems_.get())[idModel].bodyInfoVar;
      
      std::string & CharacterName = (*aListOfModelItems_.get())[idModel].name;
      dynamicsSimulator_->getCharacterSensorState(CharacterName.c_str(), 
						  sstate);
      for(unsigned int aLinkDataType=(unsigned int)
	    DynamicsSimulator::JOINT_VALUE;
	  aLinkDataType<=(unsigned int)DynamicsSimulator::ABS_ACCELERATION;
	  aLinkDataType++)
	{
	  int lid = aLinkDataType-DynamicsSimulator::JOINT_VALUE;
	  unsigned int sizeofdata[7] = {1,1,1,1,12,6,6};
	  LinkInfoSequence_var aLinkInfo = aBodyInfo->links();
	  std::vector<double>  & avec_data = listOfLogs_[idModel].jointData[lid];
	  avec_data.resize(aLinkInfo->length()*sizeofdata[lid]);

	  for(unsigned int idLink=0;idLink<aLinkInfo->length();idLink++)
	    {
	      DblSequence_var wdata;
	      dynamicsSimulator_->getCharacterLinkData(CharacterName.c_str(),
						       aLinkInfo[idLink].name,
						       (DynamicsSimulator::LinkDataType)aLinkDataType,
						       wdata);
	      for(unsigned int i=0;i<wdata->length();i++)
		avec_data[sizeofdata[lid]*idLink+i] = wdata[i];
	    }
	}
      listOfLogs_[idModel].saveLog(ltime,sstate);
    }
}
