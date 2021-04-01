#include <memory>
#include <mutex>
#include <ostream>
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
#include "nist_gear/AssemblyPlugin.hh"
#include "nist_gear/ARIAC.hh"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the AssemblyPlugin class
  struct AssemblyPluginPrivate
  {
    /// \brief Model that contains this gripper.
  public:
    physics::ModelPtr model;

    /// \brief Pointer to the world.
  public:
    physics::WorldPtr world;

    /// \brief A fixed joint to connect the gripper to an object.
  public:
    physics::JointPtr fixedJoint;

    /// \brief The assembly surface link.
  public:
    physics::LinkPtr assembly_surface_link;

    /// \brief Connection event.
  public:
    event::ConnectionPtr connection;

    /// \brief The collision for the surface link.
  public:
    std::map<std::string, physics::CollisionPtr> collisions;

    /// \brief The current contacts.
  public:
    std::vector<msgs::Contact> contacts;

    /// \brief Mutex used to protect reading/writing the sonar message.
  public:
    std::mutex mutex;

    /// \brief True if the assembly surface has an object.
  public:
    bool attached = false;

    /// \brief Rate at which to update the plugin.
  public:
    common::Time updateRate;

    /// \brief Previous time when the plugin was updated.
  public:
    common::Time prevUpdateTime;

    /// \brief Number of iterations the surface was contacting the same
    /// object.
  public:
    int posCount;

    /// \brief Number of iterations the surface was not contacting the same
    /// object.
  public:
    int zeroCount;

    /// \brief Minimum number of links touching.
  public:
    unsigned int minContactCount;

    /// \brief Steps touching before engaging fixed joint
  public:
    int attachSteps;

    /// \brief Name of the model.
  public:
    std::string name;

    /// \brief Node for communication.
  public:
    transport::NodePtr node;

    /// \brief Subscription to contact messages from the physics engine.
  public:
    transport::SubscriberPtr contactSub;

    /// \brief Whether to grip all models or only specific types.
  public:
    bool onlyGrippableModels = false;

    /// \brief Whitelist of the grippable model types to detect
  public:
    std::vector<std::string> grippableModelTypes;

    /// \brief Attached model type.
  public:
    std::string attachedObjType;

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

GZ_REGISTER_MODEL_PLUGIN(AssemblyPlugin)

/////////////////////////////////////////////////
AssemblyPlugin::AssemblyPlugin()
    : dataPtr(new AssemblyPluginPrivate)
{
  gzmsg << "Assembly plugin loaded" << std::endl;

  this->dataPtr->attached = false;
  this->dataPtr->updateRate = common::Time(0, common::Time::SecToNano(0.1));
}

/////////////////////////////////////////////////
AssemblyPlugin::~AssemblyPlugin()
{
  if (this->dataPtr->world && this->dataPtr->world->Running())
  {
    auto mgr = this->dataPtr->world->Physics()->GetContactManager();
    auto filter_name = this->dataPtr->model->GetName() + this->dataPtr->name;
    mgr->RemoveFilter(filter_name);
  }
}

/////////////////////////////////////////////////
void AssemblyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;
  this->dataPtr->world = this->dataPtr->model->GetWorld();

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->world->Name());
  this->dataPtr->name = _sdf->Get<std::string>("name");

  // Create the joint that will attach the objects to the assembly
  this->dataPtr->fixedJoint =
      this->dataPtr->world->Physics()->CreateJoint(
          "fixed", this->dataPtr->model);
  this->dataPtr->fixedJoint->SetName(this->dataPtr->model->GetName() +
                                     "__assembly_fixed_joint__" + this->dataPtr->name);

  // Read the SDF parameters
  sdf::ElementPtr contactCheck = _sdf->GetElement("contact_check");
  this->dataPtr->minContactCount =
      contactCheck->Get<unsigned int>("min_contact_count");
  this->dataPtr->attachSteps = contactCheck->Get<int>("attach_steps");
  sdf::ElementPtr assemblySurfaceLinkElem = _sdf->GetElement("assembly_surface_link");
  this->dataPtr->assembly_surface_link =
      this->dataPtr->model->GetLink(assemblySurfaceLinkElem->Get<std::string>());
  if (!this->dataPtr->assembly_surface_link)
  {
    gzerr << "Assembly surface link [" << assemblySurfaceLinkElem->Get<std::string>()
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

  // Find out the collision elements of the assembly surface 
  
  for (auto j = 0u; j < this->dataPtr->assembly_surface_link->GetChildCount(); ++j)
  {
    physics::CollisionPtr collision =
        this->dataPtr->assembly_surface_link->GetCollision(j);
    std::map<std::string, physics::CollisionPtr>::iterator collIter = this->dataPtr->collisions.find(collision->GetScopedName());
    if (collIter != this->dataPtr->collisions.end())
      continue;

    this->dataPtr->collisions[collision->GetScopedName()] = collision;
    gzdbg << collision->GetScopedName() << std::endl;
  }

  if (!this->dataPtr->collisions.empty())
  {
    // Create a filter to receive collision information
    auto mgr = this->dataPtr->world->Physics()->GetContactManager();
    auto filter_name = this->dataPtr->model->GetName() + this->dataPtr->name;
    auto topic = mgr->CreateFilter(filter_name, this->dataPtr->collisions);
    if (!this->dataPtr->contactSub)
    {
      this->dataPtr->contactSub = this->dataPtr->node->Subscribe(topic,
                                                                 &AssemblyPlugin::OnContacts, this);
    }
  }

  this->Reset();

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&AssemblyPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void AssemblyPlugin::Reset()
{
  this->dataPtr->prevUpdateTime = this->dataPtr->world->SimTime();
  this->dataPtr->zeroCount = 0;
  this->dataPtr->posCount = 0;
  this->dataPtr->attached = false;
}

/////////////////////////////////////////////////
void AssemblyPlugin::OnUpdate()
{
  this->Publish();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->world->SimTime() -
              this->dataPtr->prevUpdateTime <
          this->dataPtr->updateRate)
  {
    return;
  }

  bool modelInContact = this->CheckModelContact();
  if (modelInContact)
  {
    this->HandleAttach();
  }

  this->dataPtr->prevUpdateTime = this->dataPtr->world->SimTime();
}

/////////////////////////////////////////////////
void AssemblyPlugin::OnContacts(ConstContactsPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->contacts.clear();
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    CollisionPtr collision1 = boost::dynamic_pointer_cast<Collision>(
        this->dataPtr->world->EntityByName(_msg->contact(i).collision1()));
    CollisionPtr collision2 = boost::dynamic_pointer_cast<Collision>(
        this->dataPtr->world->EntityByName(_msg->contact(i).collision2()));

    /*if ((collision1 && !collision1->IsStatic()) &&
        (collision2 && !collision2->IsStatic()))
    {
      this->dataPtr->contacts.push_back(_msg->contact(i));
    }*/
    this->dataPtr->contacts.push_back(_msg->contact(i));
  }
}

/////////////////////////////////////////////////
bool AssemblyPlugin::GetContactNormal()
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
    gzdbg << "The assembly was in collision with its own model.\n";
  }

  return false;
}

/////////////////////////////////////////////////
void AssemblyPlugin::HandleAttach()
{
  if (this->dataPtr->attached)
  {
    return;
  }
  this->dataPtr->attached = true;

  this->dataPtr->fixedJoint->Load(this->dataPtr->assembly_surface_link,
                                  this->dataPtr->modelCollision->GetLink(), ignition::math::Pose3d());
  this->dataPtr->fixedJoint->Init();

  auto modelPtr = this->dataPtr->modelCollision->GetLink()->GetModel();
  auto name = modelPtr->GetName();
  std::string objectType = ariac::DetermineModelType(name);
  gzdbg << "Product attached to gripper: " << objectType << " named " << name << std::endl;
}

/////////////////////////////////////////////////
bool AssemblyPlugin::CheckModelContact()
{
  bool modelInContact = false;
  if (this->dataPtr->contacts.size() > 0)
  {
    gzdbg << "Number of collisions with surface: " << this->dataPtr->contacts.size() << std::endl;
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
    auto gripperLinkPose = this->dataPtr->assembly_surface_link->WorldPose();
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
void AssemblyPlugin::Publish() const
{
}
