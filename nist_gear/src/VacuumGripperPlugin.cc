/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <memory>
#include <mutex>
#include <ostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include "nist_gear/VacuumGripperPlugin.hh"
#include "nist_gear/ARIAC.hh"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the VacuumGripperPlugin class
  struct VacuumGripperPluginPrivate
  {
    /// \brief Class to store information about an object to be dropped.
    /// If the attached object is scheduled to be dropped, the drop will
    /// occur when the object enters inside the dropRegion. The object will
    /// be relocated to the respective pose.
  public:
    class DropObject
    {
      /// \brief Equality operator, result = this == _obj
      /// \param[in] _obj Object to check for equality
      /// \return true if this == _obj
    public:
      bool operator==(const DropObject &_obj) const
      {
        return this->type == _obj.type &&
               this->dropRegion == _obj.dropRegion &&
               this->destination == _obj.destination;
        this->frame == _obj.frame;
      }

      /// \brief Stream insertion operator.
      /// \param[in] _out output stream
      /// \param[in] _obj object to output
      /// \return The output stream
    public:
      friend std::ostream &operator<<(std::ostream &_out,
                                      const DropObject &_obj)
      {
        _out << _obj.type << std::endl;
        _out << _obj.dropRegion << std::endl;
        _out << "  Dst: [" << _obj.destination << "]" << std::endl;
        return _out;
      }

      /// \brief Object type.
    public:
      std::string type;

      /// \brief Object type.
    public:
      ignition::math::Box dropRegion;

      /// \brief Destination where objects are teleported to after a drop
    public:
      ignition::math::Pose3d destination;

      /// \brief Reference frame of the drop region/destination
    public:
      physics::EntityPtr frame;

      /// \brief Getter for the type of object to drop
    public:
      std::string getType() const
      {
        return this->type;
      };
    };

  public:
    ros::Publisher drop_object_publisher;
    ros::Subscriber drop_object_subscriber;
    /// \brief Collection of objects that have been dropped.
  public:
    std::vector<std::string> dropped_objects;

    /// \brief Collection of objects to be dropped.
  public:
    std::vector<DropObject> objects_to_drop;
    std::vector<nist_gear::DropProduct> object_to_drop_from_topic;

    /// \brief Model that contains this gripper.
  public:
    physics::ModelPtr model;

    /// \brief Pointer to the world.
  public:
    physics::WorldPtr world;

    /// \brief A fixed joint to connect the gripper to an object.
  public:
    physics::JointPtr fixedJoint;

    /// \brief The suction cup link.
  public:
    physics::LinkPtr suction_cup_link;

    /// \brief Connection event.
  public:
    event::ConnectionPtr connection;

    /// \brief The collisions for the links in the gripper.
  public:
    std::map<std::string, physics::CollisionPtr> collisions;

    /// \brief The current contacts.
  public:
    std::vector<msgs::Contact> contacts;

    /// \brief Mutex used to protect reading/writing the sonar message.
  public:
    std::mutex mutex;

    /// \brief True if the gripper has an object.
  public:
    bool attached = false;

    /// \brief Rate at which to update the gripper.
  public:
    common::Time updateRate;

    /// \brief Previous time when the gripper was updated.
  public:
    common::Time prevUpdateTime;

    /// \brief Number of iterations the gripper was contacting the same
    /// object.
  public:
    int posCount;

    /// \brief Number of iterations the gripper was not contacting the same
    /// object.
  public:
    int zeroCount;

    /// \brief Minimum number of links touching.
  public:
    unsigned int minContactCount;

    /// \brief Steps touching before engaging fixed joint
  public:
    int attachSteps;

    /// \brief Steps not touching before disengaging fixed joint
  public:
    int detachSteps;

    /// \brief Name of the gripper.
  public:
    std::string name;

    /// \brief Node for communication.
  public:
    transport::NodePtr node;
    // std::unique_ptr<ros::NodeHandle> rosnode;
    ros::NodeHandle *rosnode;

    /// \brief Subscription to contact messages from the physics engine.
  public:
    transport::SubscriberPtr contactSub;

    /// \brief Whether the suction is enabled or not.
  public:
    bool enabled = false;

    /// \brief Whether disabling of the suction has been requested or not.
  public:
    bool disableRequested = false;

    /// \brief Whether there's an ongoing drop.
  public:
    bool dropPending = false;

    /// \brief Whether to grip all models or only specific types.
  public:
    bool onlyGrippableModels = false;

    /// \brief Whitelist of the grippable model types to detect
  public:
    std::vector<std::string> grippableModelTypes;

    /// \brief Attached model type.
  public:
    std::string attachedObjType;

    /// \brief Attached model to be dropped.
  public:
    physics::ModelPtr dropAttachedModel;

    /// \brief Collision with the model in contact.
  public:
    physics::CollisionPtr modelCollision;

    /// \brief Normal of the contact with the model in collision.
  public:
    ignition::math::Vector3d modelContactNormal;
  };
}

using namespace gazebo;
using namespace physics;

GZ_REGISTER_MODEL_PLUGIN(VacuumGripperPlugin)

/////////////////////////////////////////////////
VacuumGripperPlugin::VacuumGripperPlugin()
    : dataPtr(new VacuumGripperPluginPrivate)
{
  this->dataPtr->attached = false;
  this->dataPtr->updateRate = common::Time(0, common::Time::SecToNano(0.1));
}

/////////////////////////////////////////////////
VacuumGripperPlugin::~VacuumGripperPlugin()
{
  if (this->dataPtr->world && this->dataPtr->world->Running())
  {
    auto mgr = this->dataPtr->world->Physics()->GetContactManager();
    mgr->RemoveFilter(this->Name());
  }
  this->dataPtr->rosnode->shutdown();
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;
  this->dataPtr->world = this->dataPtr->model->GetWorld();

  gzmsg << "VacuumGripper plugin loaded for: " << this->dataPtr->model->GetName() << " " << std::endl;

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->world->Name());
  this->dataPtr->name = _sdf->Get<std::string>("name");

  std::string robotNamespace = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    robotNamespace = _sdf->GetElement(
                             "robot_namespace")
                         ->Get<std::string>() +
                     "/";
  }

  // gzdbg << "Namespace -- " << robotNamespace << "\n";

  // initialize ROS
  this->dataPtr->rosnode = new ros::NodeHandle(this->dataPtr->model->GetName() + "/vacuumplugin");
  // this->dataPtr->rosnode.reset(new ros::NodeHandle(robotNamespace+"/vacuumplugin"));

  this->dataPtr->drop_object_publisher =
      this->dataPtr->rosnode->advertise<nist_gear::DropProducts>("/ariac/drop_products", 1000, false);

  this->dataPtr->drop_object_subscriber =
      this->dataPtr->rosnode->subscribe("/ariac/drop_products", 1000,
                                        &VacuumGripperPlugin::OnDropObjectContent, this);

  // build a message of products to drop
  nist_gear::DropProducts msgDropProducts;

  // Create the joint that will attach the objects to the suction cup
  this->dataPtr->fixedJoint =
      this->dataPtr->world->Physics()->CreateJoint(
          "fixed", this->dataPtr->model);
  this->dataPtr->fixedJoint->SetName(this->dataPtr->model->GetName() +
                                     "__vacuum_gripper_fixed_joint__");

  // Read the SDF parameters
  sdf::ElementPtr graspCheck = _sdf->GetElement("grasp_check");
  this->dataPtr->minContactCount =
      graspCheck->Get<unsigned int>("min_contact_count");
  this->dataPtr->attachSteps = graspCheck->Get<int>("attach_steps");
  this->dataPtr->detachSteps = graspCheck->Get<int>("detach_steps");
  sdf::ElementPtr suctionCupLinkElem = _sdf->GetElement("suction_cup_link");
  this->dataPtr->suction_cup_link =
      this->dataPtr->model->GetLink(suctionCupLinkElem->Get<std::string>());
  if (!this->dataPtr->suction_cup_link)
  {
    gzerr << "Suction cup link [" << suctionCupLinkElem->Get<std::string>()
          << "] not found!\n";
    return;
  }

  this->dataPtr->onlyGrippableModels = false;
  if (_sdf->HasElement("grippable_model_types"))
  {
    this->dataPtr->onlyGrippableModels = true;
    this->dataPtr->grippableModelTypes.clear();
    sdf::ElementPtr grippableModelTypesElem = _sdf->GetElement("grippable_model_types");
    if (!grippableModelTypesElem->HasElement("type"))
    {
      gzerr << "Unable to find <type> elements in the <grippable_model_types> section\n";
      return;
    }
    sdf::ElementPtr grippableModelTypeElem = grippableModelTypesElem->GetElement("type");
    while (grippableModelTypeElem)
    {
      // Parse the model type, which is encoded in model names.
      std::string type = grippableModelTypeElem->Get<std::string>();

      gzdbg << "New grippable model type: " << type << "\n";
      this->dataPtr->grippableModelTypes.push_back(type);
      grippableModelTypeElem = grippableModelTypeElem->GetNextElement("type");
    }
  }

  if (_sdf->HasElement("drops"))
  {
    sdf::ElementPtr dropsElem = _sdf->GetElement("drops");

    if (!dropsElem->HasElement("drop_regions"))
    {
      gzerr << "VacuumGripperPlugin: Unable to find <drop_regions> element in "
            << "the <drops> section\n";
      return;
    }

    sdf::ElementPtr dropRegionsElem = dropsElem->GetElement("drop_regions");
    sdf::ElementPtr dropRegionElem = NULL;
    if (dropRegionsElem->HasElement("drop_region"))
    {
      dropRegionElem = dropRegionsElem->GetElement("drop_region");
    }
    while (dropRegionElem)
    {
      if (!dropRegionElem->HasElement("min"))
      {
        gzerr << "VacuumGripperPlugin: Unable to find <min> elements in "
              << "the <drop_region> section\n";
        return;
      }

      sdf::ElementPtr minElem = dropRegionElem->GetElement("min");
      ignition::math::Vector3d min = dropRegionElem->Get<ignition::math::Vector3d>("min");

      if (!dropRegionElem->HasElement("max"))
      {
        gzerr << "VacuumGripperPlugin: Unable to find <max> elements in "
              << "the <drop_region> section\n";
        return;
      }

      sdf::ElementPtr maxElem = dropRegionElem->GetElement("max");
      ignition::math::Vector3d max = dropRegionElem->Get<ignition::math::Vector3d>("max");

      // Parse the destination.
      if (!dropRegionElem->HasElement("destination"))
      {
        gzerr << "VacuumGripperPlugin: Unable to find <destination> in "
              << "drop region\n";
        dropRegionElem = dropRegionElem->GetNextElement("drop_region");
        continue;
      }
      sdf::ElementPtr dstElement = dropRegionElem->GetElement("destination");

      // Parse the object type.
      if (!dropRegionElem->HasElement("type"))
      {
        gzerr << "VacuumGripperPlugin: Unable to find <type> in object.\n";
        dropRegionElem = dropRegionElem->GetNextElement("drop_region");
        continue;
      }

      // Parse the frame of the drop.
      physics::EntityPtr dropFrame = NULL;
      std::string dropFrameName{};

      if (dropRegionElem->HasElement("frame"))
      {
        dropFrameName = dropRegionElem->Get<std::string>("frame");
        dropFrame = this->dataPtr->world->EntityByName(dropFrameName);
        if (!dropFrame)
        {
          gzthrow(std::string("The frame '") + dropFrameName + "' does not exist");
        }
        if (!dropFrame->HasType(physics::Base::LINK) &&
            !dropFrame->HasType(physics::Base::MODEL))
        {
          gzthrow("'frame' tag must list the name of a link or model");
        }
      }

      sdf::ElementPtr typeElement = dropRegionElem->GetElement("type");
      std::string type = typeElement->Get<std::string>();

      ignition::math::Box dropRegion = ignition::math::Box(min, max);
      ignition::math::Pose3d destination = dstElement->Get<ignition::math::Pose3d>();

      nist_gear::DropProduct msgDropProduct;
      msgDropProduct.type = type;
      msgDropProduct.status = false;  // this product hasn't been dropped yet
      msgDropProduct.frame = dropFrameName;

      msgDropProducts.drop_products.push_back(msgDropProduct);

      VacuumGripperPluginPrivate::DropObject dropObject{type, dropRegion, destination, dropFrame};
      this->dataPtr->objects_to_drop.push_back(dropObject);

      dropRegionElem = dropRegionElem->GetNextElement("drop_region");
    }
    // publish this message
    // for (auto product : msgDropProducts.drop_products)
    // {
    //   gzdbg << "drop type: " << product.type << "\n";
    //   gzdbg << "drop frame: " << product.frame << "\n";
    //   gzdbg << "drop status: " << product.status << "\n";
    // }

    while (this->dataPtr->drop_object_publisher.getNumSubscribers() < 1)
    {
      gzdbg << "wait for a connection to publisher"
            << "\n";
    }
    this->dataPtr->drop_object_publisher.publish(msgDropProducts);
  }

  // Find out the collision elements of the suction cup
  for (auto j = 0u; j < this->dataPtr->suction_cup_link->GetChildCount(); ++j)
  {
    physics::CollisionPtr collision =
        this->dataPtr->suction_cup_link->GetCollision(j);
    std::map<std::string, physics::CollisionPtr>::iterator collIter =
        this->dataPtr->collisions.find(collision->GetScopedName());
    if (collIter != this->dataPtr->collisions.end())
      continue;

    this->dataPtr->collisions[collision->GetScopedName()] = collision;
  }

  if (!this->dataPtr->collisions.empty())
  {
    // Create a filter to receive collision information
    auto mgr = this->dataPtr->world->Physics()->GetContactManager();
    auto topic = mgr->CreateFilter(this->Name(), this->dataPtr->collisions);
    if (!this->dataPtr->contactSub)
    {
      this->dataPtr->contactSub = this->dataPtr->node->Subscribe(topic,
                                                                 &VacuumGripperPlugin::OnContacts, this);
    }
  }

  this->Reset();

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&VacuumGripperPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::OnDropObjectContent(nist_gear::DropProducts::ConstPtr msgDropProducts)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto msg = msgDropProducts->drop_products;
  this->dataPtr->object_to_drop_from_topic.clear();

  for (const auto &product : msg)
  {
    nist_gear::DropProduct msgDropProduct;
    msgDropProduct.type = product.type;
    msgDropProduct.frame = product.frame;
    msgDropProduct.status = product.status;

    this->dataPtr->object_to_drop_from_topic.push_back(msgDropProduct);
  }
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::Reset()
{
  this->dataPtr->prevUpdateTime = this->dataPtr->world->SimTime();
  this->dataPtr->zeroCount = 0;
  this->dataPtr->posCount = 0;
  this->dataPtr->attached = false;
  this->dataPtr->enabled = false;
}

/////////////////////////////////////////////////
std::string VacuumGripperPlugin::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
bool VacuumGripperPlugin::Enabled() const
{
  return this->dataPtr->enabled;
}

/////////////////////////////////////////////////
bool VacuumGripperPlugin::Attached() const
{
  return this->dataPtr->attached;
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::Enable()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->enabled = true;
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::Disable()
{
  // Since we can't know what thread this gets called from, just set a flag
  // and the joint will be detached in the next OnUpdate callback in the physics thread.
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->disableRequested = true;
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::OnUpdate()
{
  this->Publish();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->disableRequested)
  {
    this->HandleDetach();
    this->dataPtr->enabled = false;
    this->dataPtr->disableRequested = false;
  }

  if (this->dataPtr->world->SimTime() -
              this->dataPtr->prevUpdateTime <
          this->dataPtr->updateRate ||
      !this->dataPtr->enabled)
  {
    return;
  }

  bool modelInContact = this->CheckModelContact();
  if (modelInContact)
  {
    this->HandleAttach();
  }

  if (this->dataPtr->attached && this->dataPtr->dropPending)
  {
    for (const auto dropObject : this->dataPtr->objects_to_drop)
    {
      if (dropObject.type != this->dataPtr->attachedObjType)
      {
        continue;
      }

      // Check that dropAttachedModel is of attachedObjType
      auto name = this->dataPtr->dropAttachedModel->GetName();
      std::string objectType = ariac::DetermineModelType(name);
      if (objectType != this->dataPtr->attachedObjType)
      {
        gzdbg << "dropAttachedModel and attachedObjType are different: " << objectType << " and "
              << this->dataPtr->attachedObjType << std::endl;
        continue;
      }

      if (objectType != this->dataPtr->attachedObjType)
      {
        gzdbg << "dropAttachedModel and attachedObjType are different: " << objectType << " and "
              << this->dataPtr->attachedObjType << std::endl;
        continue;
      }

      auto objPose = this->dataPtr->dropAttachedModel->WorldPose();
      ignition::math::Pose3d dropFramePose;
      ignition::math::Matrix4d dropFrameTransMat;
      if (dropObject.frame)
      {
        // Transform the pose of the object from world frame to the specified frame.
        dropFramePose = dropObject.frame->WorldPose();
        dropFrameTransMat = ignition::math::Matrix4d(dropFramePose);
        ignition::math::Matrix4d objPoseWorld(objPose);
        objPose = (dropFrameTransMat.Inverse() * objPoseWorld).Pose();
      }

      if (!dropObject.dropRegion.Contains(objPose.Pos()))
      {
        continue;
      }

      // Drop the object.
      this->HandleDetach();

      auto objDest = dropObject.destination;
      if (dropObject.frame)
      {
        // Determine the destination in the world frame.
        ignition::math::Matrix4d objDestLocal(objDest);
        objDest = (dropFrameTransMat * objDestLocal).Pose();
      }

      if (!this->dataPtr->object_to_drop_from_topic.empty())
      {
        for (const auto &product : this->dataPtr->object_to_drop_from_topic)
        {
          nist_gear::DropProduct msgDropProduct;
          msgDropProduct.type = product.type;
          msgDropProduct.frame = product.frame;
          msgDropProduct.status = product.status;

          // gzdbg << "drop type: " << product.type << "\n";
          // gzdbg << "drop frame: " << product.frame << "\n";
          // gzdbg << "drop status: " << product.status << "\n";

          // gzdbg << "DROP: " << objectType << " " << dropObject.frame->GetName() << "\n";

          if (objectType == msgDropProduct.type &&
              msgDropProduct.frame.find(dropObject.frame->GetName()) != std::string::npos && // e.g., kit_tray_3 is in agv3::kit_tray_3
              !msgDropProduct.status)
          {
            // Teleport it to the destination.
            this->dataPtr->dropAttachedModel->SetWorldPose(objDest);
            this->dataPtr->dropAttachedModel->SetLinearVel(ignition::math::Vector3d::Zero);
            this->dataPtr->dropAttachedModel->SetLinearAccel(ignition::math::Vector3d::Zero);

            this->dataPtr->dropped_objects.push_back(this->dataPtr->attachedObjType);

            this->dataPtr->dropPending = false;
            gzdbg << "Object dropped and teleported" << std::endl;


            // update the message with status=true for the part that was just dropped
            nist_gear::DropProducts updatedDropProducts;
            for (const auto &product : this->dataPtr->object_to_drop_from_topic)
            {
              nist_gear::DropProduct msgDropProduct;
              msgDropProduct.type = product.type;
              msgDropProduct.frame = product.frame;
              if (msgDropProduct.type == objectType &&
                  msgDropProduct.frame.find(dropObject.frame->GetName()) != std::string::npos)
              {
                msgDropProduct.status = true;
              }
              updatedDropProducts.drop_products.push_back(msgDropProduct);
            }
            while (this->dataPtr->drop_object_publisher.getNumSubscribers() < 1)
            {
              gzdbg << "wait for a connection to publisher"
                    << "\n";
            }
            this->dataPtr->drop_object_publisher.publish(updatedDropProducts);
            break;
          }
        }
      }
    }
  }

  // else if (this->dataPtr->zeroCount > this->dataPtr->detachSteps &&
  //          this->dataPtr->attached)
  // {
  //   this->HandleDetach();
  // }

  this->dataPtr->prevUpdateTime = this->dataPtr->world->SimTime();
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::OnContacts(ConstContactsPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->contacts.clear();
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    CollisionPtr collision1 = boost::dynamic_pointer_cast<Collision>(
        this->dataPtr->world->EntityByName(_msg->contact(i).collision1()));
    CollisionPtr collision2 = boost::dynamic_pointer_cast<Collision>(
        this->dataPtr->world->EntityByName(_msg->contact(i).collision2()));

    if ((collision1 && !collision1->IsStatic()) &&
        (collision2 && !collision2->IsStatic()))
    {
      this->dataPtr->contacts.push_back(_msg->contact(i));
    }
  }
}

/////////////////////////////////////////////////
bool VacuumGripperPlugin::GetContactNormal()
{
  physics::CollisionPtr collisionPtr;
  ignition::math::Vector3d contactNormal;

  // Get the pointer to the collision that's not the gripper's.
  // This function is only called from the OnUpdate function so
  // the call to contacts.clear() is not going to happen in
  // parallel with the reads in the following code, no mutex needed.
  for (unsigned int i = 0; i < this->dataPtr->contacts.size(); ++i)
  {
    std::string name1 = this->dataPtr->contacts[i].collision1();
    std::string name2 = this->dataPtr->contacts[i].collision2();
    gzdbg << "Collision between '" << name1 << "' and '" << name2 << "'\n";

    if (this->dataPtr->collisions.find(name1) ==
        this->dataPtr->collisions.end())
    {
      // Model in contact is the second name
      this->dataPtr->modelCollision = boost::dynamic_pointer_cast<Collision>(
          this->dataPtr->world->EntityByName(name1));
      this->dataPtr->modelContactNormal = -1 * msgs::ConvertIgn(this->dataPtr->contacts[i].normal(0));
      return true;
    }

    if (this->dataPtr->collisions.find(name2) ==
        this->dataPtr->collisions.end())
    {
      // Model in contact is the first name -- frames are reversed
      this->dataPtr->modelCollision = boost::dynamic_pointer_cast<Collision>(
          this->dataPtr->world->EntityByName(name2));
      this->dataPtr->modelContactNormal = msgs::ConvertIgn(this->dataPtr->contacts[i].normal(0));
      return true;
    }
  }

  if (!collisionPtr)
  {
    gzdbg << "The gripper was in collision with its own model.\n";
  }

  return false;
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::HandleAttach()
{
  if (this->dataPtr->attached)
  {
    return;
  }
  this->dataPtr->attached = true;

  this->dataPtr->fixedJoint->Load(this->dataPtr->suction_cup_link,
                                  this->dataPtr->modelCollision->GetLink(), ignition::math::Pose3d());
  this->dataPtr->fixedJoint->Init();

  auto modelPtr = this->dataPtr->modelCollision->GetLink()->GetModel();
  auto name = modelPtr->GetName();
  std::string objectType = ariac::DetermineModelType(name);
  gzdbg << "Product attached to gripper: " << objectType << " named " << name << std::endl;

  // Clear these by default, they'll get set if needed later this function
  this->dataPtr->dropPending = false;
  this->dataPtr->dropAttachedModel.reset();

  // Check if the object should drop.
  auto it = find_if(this->dataPtr->objects_to_drop.begin(), this->dataPtr->objects_to_drop.end(),
                    [&objectType](const VacuumGripperPluginPrivate::DropObject &obj) {
                      return obj.getType() == objectType;
                    });
  this->dataPtr->attachedObjType = objectType;
  bool objectToBeDropped = it != this->dataPtr->objects_to_drop.end();

  if (!objectToBeDropped)
  {
    return;
  }
  auto found = std::find(std::begin(this->dataPtr->dropped_objects),
                         std::end(this->dataPtr->dropped_objects), this->dataPtr->attachedObjType);
  bool alreadyDropped = found != std::end(this->dataPtr->dropped_objects);
  if (!alreadyDropped)
  {
    this->dataPtr->dropPending = true;
    this->dataPtr->dropAttachedModel = modelPtr;
    gzdbg << "Drop scheduled" << std::endl;
  }
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::HandleDetach()
{
  gzdbg << "Detaching product from gripper." << std::endl;
  this->dataPtr->attached = false;
  this->dataPtr->fixedJoint->Detach();
}

/////////////////////////////////////////////////
bool VacuumGripperPlugin::CheckModelContact()
{
  bool modelInContact = false;
  if (this->dataPtr->contacts.size() > 0)
  {
    gzdbg << "Number of collisions with gripper: " << this->dataPtr->contacts.size() << std::endl;
  }
  if (this->dataPtr->contacts.size() >= this->dataPtr->minContactCount)
  {
    gzdbg << "More collisions than the minContactCount: " << this->dataPtr->minContactCount << std::endl;
    this->dataPtr->posCount++;
    this->dataPtr->zeroCount = 0;
  }
  else
  {
    this->dataPtr->zeroCount++;
    this->dataPtr->posCount = std::max(0, this->dataPtr->posCount - 1);
  }

  if (this->dataPtr->posCount > this->dataPtr->attachSteps &&
      !this->dataPtr->attached)
  {

    if (!this->GetContactNormal())
    {
      return false;
    }

    if (this->dataPtr->onlyGrippableModels)
    {
      // Only attach whitelisted models
      auto modelPtr = this->dataPtr->modelCollision->GetLink()->GetModel();
      auto modelName = modelPtr->GetName();
      gzdbg << "Product in contact with gripper: " << modelName << std::endl;
      std::string modelType = ariac::DetermineModelType(modelName);
      auto it = std::find(this->dataPtr->grippableModelTypes.begin(), this->dataPtr->grippableModelTypes.end(), modelType);
      bool grippableModel = it != this->dataPtr->grippableModelTypes.end();
      if (!grippableModel)
      {
        gzdbg << "Not a grippable type." << std::endl;
        return false;
      }
    }

    // Only consider models with collision normals aligned with the normal of the gripper
    auto gripperLinkPose = this->dataPtr->suction_cup_link->WorldPose();
    ignition::math::Vector3d gripperLinkNormal =
        gripperLinkPose.Rot().RotateVector(ignition::math::Vector3d(0, 0, 1));
    double alignment = gripperLinkNormal.Dot(this->dataPtr->modelContactNormal);

    // Alignment of > 0.95 represents alignment angle of < acos(0.95) = ~18 degrees
    if (alignment > 0.95)
    {
      modelInContact = true;
    }
  }
  return modelInContact;
}

/////////////////////////////////////////////////
void VacuumGripperPlugin::Publish() const
{
}
