#include <sim_ros2_interface.h>
#include <simPlusPlus/Plugin.h>
#include <simPlusPlus/Handle.h>

#include <cstdlib>
#include <functional>
using namespace std::placeholders;

#include <boost/type_erasure/any_cast.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.hpp>
#if image_transport_FOUND
#include <image_transport/image_transport.hpp>
#endif
//#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

struct unsupported_type : public std::exception
{
    std::string message;
    unsupported_type(const std::string &w, const std::string &t)
    {
        std::stringstream ss;
        ss << "Unsupported " << w << " type '" << t << "'. You may want to add it to meta/interfaces.txt and recompile the ROS2Interface plugin.";
        message = ss.str();
    }
    ~unsupported_type() throw() {}
    const char * what() const throw() {return message.c_str();}
};

class Plugin : public sim::Plugin
{
public:
    void onStart()
    {
        if(!initialize())
            throw std::runtime_error("failed to initialize ROS2 node");

        if(!registerScriptStuff())
            throw std::runtime_error("failed to register script stuff");

        setExtVersion("ROS2 Interface Plugin");
        setBuildDate(BUILD_DATE);
    }

    void onEnd()
    {
        shutdown();
    }

    void onInstancePass(const sim::InstancePassFlags &flags, bool first)
    {
        rclcpp::spin_some(node);
    }

    void onMainScriptAboutToBeCalled(int &out)
    {

        int stopSimulationRequestCounter;
        simGetIntegerParameter(sim_intparam_stop_request_counter, &stopSimulationRequestCounter);
        simBool doNotRun = simGetBoolParameter(sim_boolparam_rosinterface_donotrunmainscript);
        if(doNotRun > 0)
        {
            if(previousStopSimulationRequestCounter == -1)
                previousStopSimulationRequestCounter = stopSimulationRequestCounter;
            if(previousStopSimulationRequestCounter == stopSimulationRequestCounter)
                out = 0; // this tells CoppeliaSim that we don't wanna execute the main script
        }
        else
            previousStopSimulationRequestCounter = -1;
    }

    void onSimulationAboutToStart()
    {
        previousStopSimulationRequestCounter = -1;
    }

    void onScriptStateDestroyed(int scriptID)
    {
        for(auto subscriptionProxy : subscriptionHandles.find(scriptID))
        {
            if(!subscriptionProxy->subscription.empty())
            {
                shutdownSubscription_in in1;
                in1.subscriptionHandle = subscriptionProxy->handle;
                shutdownSubscription_out out1;
                shutdownSubscription(&in1, &out1);
            }
#if image_transport_FOUND
            if(subscriptionProxy->imageTransportSubscription)
            {
                imageTransportShutdownSubscription_in in1;
                in1.subscriptionHandle = subscriptionProxy->handle;
                imageTransportShutdownSubscription_out out1;
                imageTransportShutdownSubscription(&in1, &out1);
            }
#endif
        }
        for(auto publisherProxy : publisherHandles.find(scriptID))
        {
            if(!publisherProxy->publisher.empty())
            {
                shutdownPublisher_in in1;
                in1.publisherHandle = publisherProxy->handle;
                shutdownPublisher_out out1;
                shutdownPublisher(&in1, &out1);
            }
#if image_transport_FOUND
            if(publisherProxy->imageTransportPublisher)
            {
                imageTransportShutdownPublisher_in in1;
                in1.publisherHandle = publisherProxy->handle;
                imageTransportShutdownPublisher_out out1;
                imageTransportShutdownPublisher(&in1, &out1);
            }
#endif
        }
        for(auto clientProxy : clientHandles.find(scriptID))
        {
            shutdownClient_in in1;
            in1.clientHandle = clientProxy->handle;
            shutdownClient_out out1;
            shutdownClient(&in1, &out1);
        }
        for(auto serviceProxy : serviceHandles.find(scriptID))
        {
            shutdownService_in in1;
            in1.serviceHandle = serviceProxy->handle;
            shutdownService_out out1;
            shutdownService(&in1, &out1);
        }
        for(auto actionClientProxy : actionClientHandles.find(scriptID))
        {
            shutdownActionClient_in in1;
            in1.actionClientHandle = actionClientProxy->handle;
            shutdownActionClient_out out1;
            shutdownActionClient(&in1, &out1);
        }
        for(auto actionServerProxy : actionServerHandles.find(scriptID))
        {
            shutdownActionServer_in in1;
            in1.actionServerHandle = actionServerProxy->handle;
            shutdownActionServer_out out1;
            shutdownActionServer(&in1, &out1);
        }
    }

    bool shouldProxyBeDestroyedAfterSimulationStop(int scriptID)
    {
        if(simGetSimulationState() == sim_simulation_stopped)
            return false;
        int property;
        int associatedObject;
        if(simGetScriptProperty(scriptID, &property, &associatedObject) == -1)
            return false;
#if SIM_PROGRAM_FULL_VERSION_NB <= 4010003
        if(property & sim_scripttype_threaded)
            property -= sim_scripttype_threaded;
#else
        if(property & sim_scripttype_threaded_old)
            property -= sim_scripttype_threaded_old;
#endif
        if(property == sim_scripttype_addonscript || property == sim_scripttype_addonfunction || property == sim_scripttype_customizationscript)
            return false;
        return true;
    }

    static void ros_imtr_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg, SubscriptionProxy *subscriptionProxy, Plugin *plugin)
    {
        if(msg->is_bigendian)
        {
            std::cerr << "ros_imtr_callback: error: big endian image not supported" << std::endl;
            return;
        }

        int data_len = msg->step * msg->height;

        imageTransportCallback_in in_args;
        imageTransportCallback_out out_args;

        in_args.width = msg->width;
        in_args.height = msg->height;
        in_args.data.resize(data_len);

        for(unsigned int i = 0; i < msg->height; i++)
        {
            int msg_idx = (msg->height - i - 1) * msg->step;
            int buf_idx = i * msg->step;
            for(unsigned int j = 0; j < msg->step; j++)
            {
                in_args.data[buf_idx + j] = msg->data[msg_idx + j];
            }
        }

        if(!imageTransportCallback(subscriptionProxy->topicCallback.scriptId, subscriptionProxy->topicCallback.name.c_str(), &in_args, &out_args))
        {
            std::cerr << "ros_imtr_callback: error: failed to call callback" << std::endl;
            return;
        }
    }

    void createSubscription(createSubscription_in *in, createSubscription_out *out)
    {
        SubscriptionProxy *subscriptionProxy = new SubscriptionProxy();
        subscriptionProxy->topicName = in->topicName;
        subscriptionProxy->topicType = in->topicType;
        subscriptionProxy->topicCallback.scriptId = in->_.scriptID;
        subscriptionProxy->topicCallback.name = in->topicCallback;

        if(0) {}
#include <sub_new.cpp>
        else
        {
            throw unsupported_type("message", subscriptionProxy->topicType);
        }

        out->subscriptionHandle = subscriptionProxy->handle = subscriptionHandles.add(subscriptionProxy, in->_.scriptID);
    }

    void shutdownSubscription(shutdownSubscription_in *in, shutdownSubscription_out *out)
    {
        SubscriptionProxy *subscriptionProxy = subscriptionHandles.get(in->subscriptionHandle);

        if(0) {}
#include <sub_del.cpp>
        else
        {
            throw unsupported_type("message", subscriptionProxy->topicType);
        }

        delete subscriptionHandles.remove(subscriptionProxy);
    }

    void subscriptionTreatUInt8ArrayAsString(subscriptionTreatUInt8ArrayAsString_in *in, subscriptionTreatUInt8ArrayAsString_out *out)
    {
        SubscriptionProxy *subscriptionProxy = subscriptionHandles.get(in->subscriptionHandle);
        subscriptionProxy->wr_opt.uint8array_as_string = true;
    }

    void createPublisher(createPublisher_in *in, createPublisher_out *out)
    {
        PublisherProxy *publisherProxy = new PublisherProxy();
        publisherProxy->topicName = in->topicName;
        publisherProxy->topicType = in->topicType;

        if(0) {}
#include <pub_new.cpp>
        else
        {
            throw unsupported_type("message", publisherProxy->topicType);
        }

        out->publisherHandle = publisherProxy->handle = publisherHandles.add(publisherProxy, in->_.scriptID);
    }

    void shutdownPublisher(shutdownPublisher_in *in, shutdownPublisher_out *out)
    {
        PublisherProxy *publisherProxy = publisherHandles.get(in->publisherHandle);

        if(0) {}
#include <pub_del.cpp>
        else
        {
            throw unsupported_type("message", publisherProxy->topicType);
        }

        delete publisherHandles.remove(publisherProxy);
    }

    void publisherTreatUInt8ArrayAsString(publisherTreatUInt8ArrayAsString_in *in, publisherTreatUInt8ArrayAsString_out *out)
    {
        PublisherProxy *publisherProxy = publisherHandles.get(in->publisherHandle);
        publisherProxy->rd_opt.uint8array_as_string = true;
    }

    void publish(publish_in *in, publish_out *out)
    {
        PublisherProxy *publisherProxy = publisherHandles.get(in->publisherHandle);

        simMoveStackItemToTop(in->_.stackID, 0);

        if(0) {}
#include <pub_publish.cpp>
        else
        {
            throw unsupported_type("message", publisherProxy->topicType);
        }
    }

    void createClient(createClient_in *in, createClient_out *out)
    {
        ClientProxy *clientProxy = new ClientProxy();
        clientProxy->serviceName = in->serviceName;
        clientProxy->serviceType = in->serviceType;

        if(0) {}
#include <cli_new.cpp>
        else
        {
            throw unsupported_type("service", clientProxy->serviceType);
        }

        out->clientHandle = clientProxy->handle = clientHandles.add(clientProxy, in->_.scriptID);
    }

    void shutdownClient(shutdownClient_in *in, shutdownClient_out *out)
    {
        ClientProxy *clientProxy = clientHandles.get(in->clientHandle);

        if(0) {}
#include <cli_del.cpp>
        else
        {
            throw unsupported_type("service", clientProxy->serviceType);
        }

        delete clientHandles.remove(clientProxy);
    }

    void clientTreatUInt8ArrayAsString(clientTreatUInt8ArrayAsString_in *in, clientTreatUInt8ArrayAsString_out *out)
    {
        ClientProxy *clientProxy = clientHandles.get(in->clientHandle);
        clientProxy->rd_opt.uint8array_as_string = true;
        clientProxy->wr_opt.uint8array_as_string = true;
    }

    void waitForService(waitForService_in *in, waitForService_out *out)
    {
        ClientProxy *clientProxy = clientHandles.get(in->clientHandle);

        if(0) {}
#include <cli_wait.cpp>
        else
        {
            throw unsupported_type("service", clientProxy->serviceType);
        }
    }

    void call(call_in *in, call_out *out)
    {
        ClientProxy *clientProxy = clientHandles.get(in->clientHandle);

        simMoveStackItemToTop(in->_.stackID, 0);

        if(0) {}
#include <cli_call.cpp>
        else
        {
            throw unsupported_type("service", clientProxy->serviceType);
        }
    }

    void createService(createService_in *in, createService_out *out)
    {
        ServiceProxy *serviceProxy = new ServiceProxy();
        serviceProxy->serviceName = in->serviceName;
        serviceProxy->serviceType = in->serviceType;
        serviceProxy->serviceCallback.scriptId = in->_.scriptID;
        serviceProxy->serviceCallback.name = in->serviceCallback;

        if(0) {}
#include <srv_new.cpp>
        else
        {
            throw unsupported_type("service", serviceProxy->serviceType);
        }

        out->serviceHandle = serviceProxy->handle = serviceHandles.add(serviceProxy, in->_.scriptID);
    }

    void shutdownService(shutdownService_in *in, shutdownService_out *out)
    {
        ServiceProxy *serviceProxy = serviceHandles.get(in->serviceHandle);

        if(0) {}
#include <srv_del.cpp>
        else
        {
            throw unsupported_type("service", serviceProxy->serviceType);
        }

        delete serviceHandles.remove(serviceProxy);
    }

    void serviceTreatUInt8ArrayAsString(serviceTreatUInt8ArrayAsString_in *in, serviceTreatUInt8ArrayAsString_out *out)
    {
        ServiceProxy *serviceProxy = serviceHandles.get(in->serviceHandle);
        serviceProxy->rd_opt.uint8array_as_string = true;
        serviceProxy->wr_opt.uint8array_as_string = true;
    }

    void createActionClient(createActionClient_in *in, createActionClient_out *out)
    {
        ActionClientProxy *actionClientProxy = new ActionClientProxy();
        actionClientProxy->actionName = in->actionName;
        actionClientProxy->actionType = in->actionType;
        actionClientProxy->goalResponseCallback.scriptId = in->_.scriptID;
        actionClientProxy->goalResponseCallback.name = in->goalResponseCallback;
        actionClientProxy->feedbackCallback.scriptId = in->_.scriptID;
        actionClientProxy->feedbackCallback.name = in->feedbackCallback;
        actionClientProxy->resultCallback.scriptId = in->_.scriptID;
        actionClientProxy->resultCallback.name = in->resultCallback;

        if(0) {}
#include <actcli_new.cpp>
        else
        {
            throw unsupported_type("action", actionClientProxy->actionType);
        }

        out->actionClientHandle = actionClientProxy->handle = actionClientHandles.add(actionClientProxy, in->_.scriptID);
    }

    void shutdownActionClient(shutdownActionClient_in *in, shutdownActionClient_out *out)
    {
        ActionClientProxy *actionClientProxy = actionClientHandles.get(in->actionClientHandle);

        if(0) {}
#include <actcli_del.cpp>
        else
        {
            throw unsupported_type("action", actionClientProxy->actionType);
        }

        delete actionClientHandles.remove(actionClientProxy);
    }

    void actionClientTreatUInt8ArrayAsString(actionClientTreatUInt8ArrayAsString_in *in, actionClientTreatUInt8ArrayAsString_out *out)
    {
        ActionClientProxy *actionClientProxy = actionClientHandles.get(in->actionClientHandle);
        actionClientProxy->rd_opt.uint8array_as_string = true;
        actionClientProxy->wr_opt.uint8array_as_string = true;
    }

    void sendGoal(sendGoal_in *in, sendGoal_out *out)
    {
        ActionClientProxy *actionClientProxy = actionClientHandles.get(in->actionClientHandle);

        simMoveStackItemToTop(in->_.stackID, 0);

        if(0) {}
#include <actcli_sendGoal.cpp>
        else
        {
            throw unsupported_type("action", actionClientProxy->actionType);
        }
    }

    void cancelLastGoal(cancelLastGoal_in *in, cancelLastGoal_out *out)
    {
        ActionClientProxy *actionClientProxy = actionClientHandles.get(in->actionClientHandle);

        if(0) {}
#include <actcli_cancelLastGoal.cpp>
        else
        {
            throw unsupported_type("action", actionClientProxy->actionType);
        }
    }

    void createActionServer(createActionServer_in *in, createActionServer_out *out)
    {
        ActionServerProxy *actionServerProxy = new ActionServerProxy();
        actionServerProxy->actionName = in->actionName;
        actionServerProxy->actionType = in->actionType;
        actionServerProxy->handleGoalCallback.scriptId = in->_.scriptID;
        actionServerProxy->handleGoalCallback.name = in->handleGoalCallback;
        actionServerProxy->handleCancelCallback.scriptId = in->_.scriptID;
        actionServerProxy->handleCancelCallback.name = in->handleCancelCallback;
        actionServerProxy->handleAcceptedCallback.scriptId = in->_.scriptID;
        actionServerProxy->handleAcceptedCallback.name = in->handleAcceptedCallback;

        if(0) {}
#include <actsrv_new.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        out->actionServerHandle = actionServerProxy->handle = actionServerHandles.add(actionServerProxy, in->_.scriptID);
    }

    void shutdownActionServer(shutdownActionServer_in *in, shutdownActionServer_out *out)
    {
        ActionServerProxy *actionServerProxy = actionServerHandles.get(in->actionServerHandle);

        if(0) {}
#include <actsrv_del.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        delete actionServerHandles.remove(actionServerProxy);
    }

    void actionServerTreatUInt8ArrayAsString(actionServerTreatUInt8ArrayAsString_in *in, actionServerTreatUInt8ArrayAsString_out *out)
    {
        ActionServerProxy *actionServerProxy = actionServerHandles.get(in->actionServerHandle);
        actionServerProxy->rd_opt.uint8array_as_string = true;
        actionServerProxy->wr_opt.uint8array_as_string = true;
    }

    template<typename Action>
    rclcpp_action::ServerGoalHandle<Action> * getGoalHandle(ActionServerProxy *actionServerProxy, const std::string &goalUUIDstr)
    {
        return getGoalHandle<Action>(actionServerProxy, goalUUIDfromString(goalUUIDstr));
    }

    template<typename Action>
    rclcpp_action::ServerGoalHandle<Action> * getGoalHandle(ActionServerProxy *actionServerProxy, const rclcpp_action::GoalUUID &goalUUID)
    {
        auto actsrv = boost::any_cast< std::shared_ptr< rclcpp_action::Server< Action > > >(actionServerProxy->action_server);
        auto it = actionServerProxy->goalHandles.find(goalUUID);
        if(it == actionServerProxy->goalHandles.end())
            throw sim::exception("goal handle '%s' does not exist", goalUUIDtoString(goalUUID));
        auto goalHandleBase = it->second.get();
        return dynamic_cast< rclcpp_action::ServerGoalHandle<Action>* >(goalHandleBase);
    }

    void cleanupTerminalGoalHandles(ActionServerProxy *actionServerProxy)
    {
        /* FIXME: crash when actionServerActionSucceed is called */ return;

        for(auto it = actionServerProxy->goalHandles.begin(); it != actionServerProxy->goalHandles.end(); )
        {
            if(it->second->is_active())
                ++it;
            else
                actionServerProxy->goalHandles.erase(it);
        }
    }

    void actionServerPublishFeedback(actionServerPublishFeedback_in *in, actionServerPublishFeedback_out *out)
    {
        ActionServerProxy *actionServerProxy = actionServerHandles.get(in->actionServerHandle);

        if(0) {}
#include <actsrv_action_publish_feedback.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }
    }

    void actionServerActionAbort(actionServerActionAbort_in *in, actionServerActionAbort_out *out)
    {
        ActionServerProxy *actionServerProxy = actionServerHandles.get(in->actionServerHandle);

        if(0) {}
#include <actsrv_action_abort.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        cleanupTerminalGoalHandles(actionServerProxy);
    }

    void actionServerActionSucceed(actionServerActionSucceed_in *in, actionServerActionSucceed_out *out)
    {
        ActionServerProxy *actionServerProxy = actionServerHandles.get(in->actionServerHandle);

        if(0) {}
#include <actsrv_action_succeed.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        cleanupTerminalGoalHandles(actionServerProxy);
    }

    void actionServerActionCanceled(actionServerActionCanceled_in *in, actionServerActionCanceled_out *out)
    {
        ActionServerProxy *actionServerProxy = actionServerHandles.get(in->actionServerHandle);

        if(0) {}
#include <actsrv_action_canceled.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        cleanupTerminalGoalHandles(actionServerProxy);
    }

    void actionServerActionExecute(actionServerActionExecute_in *in, actionServerActionExecute_out *out)
    {
        ActionServerProxy *actionServerProxy = actionServerHandles.get(in->actionServerHandle);

        if(0) {}
#include <actsrv_action_execute.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        cleanupTerminalGoalHandles(actionServerProxy);
    }

    void actionServerActionIsCanceling(actionServerActionIsCanceling_in *in, actionServerActionIsCanceling_out *out)
    {
        ActionServerProxy *actionServerProxy = actionServerHandles.get(in->actionServerHandle);

        if(0) {}
#include <actsrv_action_is_canceling.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }
    }

    void actionServerActionIsActive(actionServerActionIsActive_in *in, actionServerActionIsActive_out *out)
    {
        ActionServerProxy *actionServerProxy = actionServerHandles.get(in->actionServerHandle);

        if(0) {}
#include <actsrv_action_is_active.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }
    }

    void actionServerActionIsExecuting(actionServerActionIsExecuting_in *in, actionServerActionIsExecuting_out *out)
    {
        ActionServerProxy *actionServerProxy = actionServerHandles.get(in->actionServerHandle);

        if(0) {}
#include <actsrv_action_is_executing.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }
    }

    void sendTransform(sendTransform_in *in, sendTransform_out *out)
    {
        geometry_msgs::msg::TransformStamped t;
        read__geometry_msgs__msg__TransformStamped(in->_.stackID, &t);
        tfbr->sendTransform(t);
    }

    void sendTransforms(sendTransforms_in *in, sendTransforms_out *out)
    {
        std::vector<geometry_msgs::msg::TransformStamped> v;

        sim::moveStackItemToTop(in->_.stackID, 0);
        int i = sim::getStackTableInfo(in->_.stackID, 0);
        if(i < 0)
            throw sim::exception("error reading input argument 1 (origin): expected array");
        int oldsz = sim::getStackSize(in->_.stackID);
        sim::unfoldStackTable(in->_.stackID);
        int sz = (sim::getStackSize(in->_.stackID) - oldsz + 1) / 2;
        for(int i = 0; i < sz; i++)
        {
            sim::moveStackItemToTop(in->_.stackID, oldsz - 1);
            int j;
            read__int32(in->_.stackID, &j);
            simMoveStackItemToTop(in->_.stackID, oldsz - 1);
            geometry_msgs::msg::TransformStamped t;
            read__geometry_msgs__msg__TransformStamped(in->_.stackID, &t);
            v.push_back(t);
        }

        tfbr->sendTransform(v);
    }

    void imageTransportCreateSubscription(imageTransportCreateSubscription_in *in, imageTransportCreateSubscription_out *out)
    {
#if image_transport_FOUND
        SubscriptionProxy *subscriptionProxy = new SubscriptionProxy();
        subscriptionProxy->topicName = in->topicName;
        subscriptionProxy->topicType = "@image_transport";
        subscriptionProxy->topicCallback.scriptId = in->_.scriptID;
        subscriptionProxy->topicCallback.name = in->topicCallback;

        subscriptionProxy->imageTransportSubscription = imtr->subscribe(in->topicName, in->queueSize, std::bind(ros_imtr_callback, _1, subscriptionProxy, this));

        if(!subscriptionProxy->imageTransportSubscription)
        {
            throw sim::exception("failed creation of ROS ImageTransport subscription");
        }

        out->subscriptionHandle = subscriptionProxy->handle = subscriptionHandles.add(subscriptionProxy, in->_.scriptID);
#else
        throw sim::exception("image_transport not available. please rebuild this plugin.");
#endif
    }

    void imageTransportShutdownSubscription(imageTransportShutdownSubscription_in *in, imageTransportShutdownSubscription_out *out)
    {
        SubscriptionProxy *subscriptionProxy = subscriptionHandles.get(in->subscriptionHandle);
        //subscriptionProxy->imageTransportSubscription.shutdown();
        delete subscriptionHandles.remove(subscriptionProxy);
    }

    void imageTransportCreatePublisher(imageTransportCreatePublisher_in *in, imageTransportCreatePublisher_out *out)
    {
#if image_transport_FOUND
        PublisherProxy *publisherProxy = new PublisherProxy();
        publisherProxy->topicName = in->topicName;
        publisherProxy->topicType = "@image_transport";

        publisherProxy->imageTransportPublisher = imtr->advertise(in->topicName, in->queueSize);

        if(!publisherProxy->imageTransportPublisher)
        {
            throw sim::exception("failed creation of ROS ImageTransport publisher");
        }

        out->publisherHandle = publisherProxy->handle = publisherHandles.add(publisherProxy, in->_.scriptID);
#else
        throw sim::exception("image_transport not available. please rebuild this plugin.");
#endif
    }

    void imageTransportShutdownPublisher(imageTransportShutdownPublisher_in *in, imageTransportShutdownPublisher_out *out)
    {
#if image_transport_FOUND
        PublisherProxy *publisherProxy = publisherHandles.get(in->publisherHandle);
        //publisherProxy->imageTransportPublisher.shutdown();
        delete publisherHandles.remove(publisherProxy);
#else
        throw sim::exception("image_transport not available. please rebuild this plugin.");
#endif
    }

    void imageTransportPublish(imageTransportPublish_in *in, imageTransportPublish_out *out)
    {
#if image_transport_FOUND
        PublisherProxy *publisherProxy = publisherHandles.get(in->publisherHandle);

        sensor_msgs::msg::Image image_msg;
        rclcpp::Clock ros_clock(RCL_ROS_TIME);
        image_msg.header.stamp = ros_clock.now();
        image_msg.header.frame_id = in->frame_id;
        image_msg.encoding = sensor_msgs::image_encodings::RGB8;
        image_msg.width = in->width;
        image_msg.height = in->height;
        image_msg.step = image_msg.width * 3;

        int data_len = image_msg.step * image_msg.height;
        image_msg.data.resize(data_len);
        image_msg.is_bigendian = 0;

        for(unsigned int i = 0; i < image_msg.height; i++)
        {
            int msg_idx = (image_msg.height - i - 1) * image_msg.step;
            int buf_idx = i * image_msg.step;
            for(unsigned int j = 0; j < image_msg.step; j++)
            {
                image_msg.data[msg_idx + j] = in->data[buf_idx + j];
            }
        }

        publisherProxy->imageTransportPublisher.publish(image_msg);
#else
        throw sim::exception("image_transport not available. please rebuild this plugin.");
#endif
    }

    void getTime(getTime_in *in, getTime_out *out)
    {
        rcl_clock_type_t t = RCL_ROS_TIME;
        switch(in->clock_type)
        {
        case sim_ros2_clock_ros:    t = RCL_ROS_TIME;    break;
        case sim_ros2_clock_system: t = RCL_SYSTEM_TIME; break;
        case sim_ros2_clock_steady: t = RCL_STEADY_TIME; break;
        }
        rclcpp::Clock ros_clock(t);
        builtin_interfaces::msg::Time ros_now = ros_clock.now();
        out->time.sec = ros_now.sec;
        out->time.nanosec = ros_now.nanosec;
    }

    void getParamString(getParamString_in *in, getParamString_out *out)
    {
        out->exists = params_client->has_parameter(in->name);
        if(out->exists)
        {
            auto &param = params_client->get_parameters({in->name}).front();
            if(param.get_type() == rclcpp::PARAMETER_STRING)
                out->value = param.get_value<rclcpp::PARAMETER_STRING>();
        }
    }

    void getParamInt(getParamInt_in *in, getParamInt_out *out)
    {
        out->exists = params_client->has_parameter(in->name);
        if(out->exists)
        {
            auto &param = params_client->get_parameters({in->name}).front();
            if(param.get_type() == rclcpp::PARAMETER_INTEGER)
                out->value = param.get_value<rclcpp::PARAMETER_INTEGER>();
        }
    }

    void getParamDouble(getParamDouble_in *in, getParamDouble_out *out)
    {
        out->exists = params_client->has_parameter(in->name);
        if(out->exists)
        {
            auto &param = params_client->get_parameters({in->name}).front();
            if(param.get_type() == rclcpp::PARAMETER_DOUBLE)
                out->value = param.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
    }

    void getParamBool(getParamBool_in *in, getParamBool_out *out)
    {
        out->exists = params_client->has_parameter(in->name);
        if(out->exists)
        {
            auto &param = params_client->get_parameters({in->name}).front();
            if(param.get_type() == rclcpp::PARAMETER_BOOL)
                out->value = param.get_value<rclcpp::PARAMETER_BOOL>();
        }
    }

    void setParamString(setParamString_in *in, setParamString_out *out)
    {
        params_client->set_parameters({rclcpp::Parameter(in->name, in->value)});
    }

    void setParamInt(setParamInt_in *in, setParamInt_out *out)
    {
        params_client->set_parameters({rclcpp::Parameter(in->name, in->value)});
    }

    void setParamDouble(setParamDouble_in *in, setParamDouble_out *out)
    {
        params_client->set_parameters({rclcpp::Parameter(in->name, in->value)});
    }

    void setParamBool(setParamBool_in *in, setParamBool_out *out)
    {
        params_client->set_parameters({rclcpp::Parameter(in->name, in->value)});
    }

    void hasParam(hasParam_in *in, hasParam_out *out)
    {
        out->exists = params_client->has_parameter(in->name);
    }

    void deleteParam(deleteParam_in *in, deleteParam_out *out)
    {
        if(params_client->has_parameter(in->name))
            params_client->set_parameters({rclcpp::Parameter(in->name, rclcpp::ParameterValue())});
    }

    void createInterface(createInterface_in *in, createInterface_out *out)
    {
        if(0) {}
#include <if_new.cpp>
        else
        {
            throw unsupported_type("interface", in->type);
        }
    }

    void getInterfaceConstants(getInterfaceConstants_in *in, getInterfaceConstants_out *out)
    {
        if(0) {}
#include <if_const.cpp>
        else
        {
            throw unsupported_type("interface", in->type);
        }
    }

    void supportedInterfaces(supportedInterfaces_in *in, supportedInterfaces_out *out)
    {
#include <if_list.cpp>
    }

    bool initialize()
    {
        rclcpp::init(0, nullptr);

        int node_name_length = 0;
        char *node_name = nullptr;
        node_name = simGetStringNamedParam("ROS2Interface.nodeName", &node_name_length);

        node = rclcpp::Node::make_shared(node_name && node_name_length ? node_name : "sim_ros2_interface");

        if(node_name) simReleaseBuffer(node_name);

        tfbr = new tf2_ros::TransformBroadcaster(node);
#if image_transport_FOUND
        imtr = new image_transport::ImageTransport(node);
#endif

        params_client = std::make_shared<rclcpp::SyncParametersClient>(node);
        while(!params_client->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                sim::addLog(sim_verbosity_errors, "Interrupted while waiting for parameters service");
                return false;
            }
            sim::addLog(sim_verbosity_debug, "Parameters service not available. Waiting for it...");
        }

        return true;
    }

    void shutdown()
    {
        rclcpp::shutdown();
        params_client = nullptr;
        node = nullptr;

#if image_transport_FOUND
        delete imtr;
#endif
        delete tfbr;
    }

private:
    int previousStopSimulationRequestCounter = -1;

    rclcpp::Node::SharedPtr node = nullptr;
    rclcpp::SyncParametersClient::SharedPtr params_client = nullptr;

    tf2_ros::TransformBroadcaster *tfbr = nullptr;
#if image_transport_FOUND
    image_transport::ImageTransport *imtr = nullptr;
#endif

    sim::Handles<SubscriptionProxy> subscriptionHandles;
    sim::Handles<PublisherProxy> publisherHandles;
    sim::Handles<ClientProxy> clientHandles;
    sim::Handles<ServiceProxy> serviceHandles;
    sim::Handles<ActionClientProxy> actionClientHandles;
    sim::Handles<ActionServerProxy> actionServerHandles;
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
