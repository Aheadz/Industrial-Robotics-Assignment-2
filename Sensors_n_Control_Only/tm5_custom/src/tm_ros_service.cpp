#include "tm_driver/tm_ros_node.h"

////////////////////////////////
// Service
////////////////////////////////

bool TmRosNode::connect_tm(tm5_custom::ConnectTMRequest &req, tm5_custom::ConnectTMResponse &res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req.timeout);
    int t_v = (int)(1000.0 * req.timeval);
    switch (req.server) {
    case tm5_custom::ConnectTMRequest::TMSVR:
        if (req.connect) {
            rb = ethernetSlaveConnection->connect(t_o);
        }
        if (req.reconnect) {
            rb = ethernetSlaveConnection->re_connect(t_o,t_v);
        }
        else {
            ethernetSlaveConnection->no_connect();
        }
        break;
    case tm5_custom::ConnectTMRequest::TMSCT:
        rb = listenNodeConnection->connect_tmsct(req.timeout, req.timeval, req.connect, req.reconnect);
    }
    res.ok = rb;
    return rb;
}

bool TmRosNode::write_item(tm5_custom::WriteItemRequest &req, tm5_custom::WriteItemResponse &res)
{
    bool rb = false;
    std::string content = req.item + "=" + req.value;
    rb = (iface_.svr.send_content_str(req.id, content) == iface_.RC_OK);
    res.ok = rb;
    return rb;
}

bool TmRosNode::ask_item(tm5_custom::AskItemRequest &req, tm5_custom::AskItemResponse &res)
{
    PubMsg &pm = pm_;
    TmSvrData &data = iface_.svr.data;
    bool rb = false;

    svr_mtx_.lock();
    svr_updated_ = false;
    svr_mtx_.unlock();

    rb = (iface_.svr.send_content(req.id, TmSvrData::Mode::READ_STRING, req.item) == iface_.RC_OK);

    {
        boost::unique_lock<boost::mutex> lck(svr_mtx_);
        if (rb && req.wait_time > 0.0) {
            if (!svr_updated_) {
                svr_cond_.wait_for(lck, boost::chrono::duration<double>(req.wait_time));
            }
            if (!svr_updated_) {
                rb = false;
            }
            res.id = pm.svr_msg.id;
            res.value = pm.svr_msg.content;
        }
        svr_updated_ = false;
    }
    res.ok = rb;
    return rb;
}
bool TmRosNode::send_script(tm5_custom::SendScriptRequest &req, tm5_custom::SendScriptResponse &res)
{   
    bool rb = listenNodeConnection->send_listen_node_script(req.id, req.script);
    res.ok = rb;
    return rb;
}

bool TmRosNode::set_event(tm5_custom::SetEventRequest &req, tm5_custom::SetEventResponse &res)
{
    bool rb = false;
    switch (req.func) {
    case tm5_custom::SetEventRequest::EXIT:
        rb = iface_.script_exit();
        break;
    case tm5_custom::SetEventRequest::TAG:
        rb = iface_.set_tag((int)(req.arg0), (int)(req.arg1));
        break;
    case tm5_custom::SetEventRequest::WAIT_TAG:
        rb = iface_.set_wait_tag((int)(req.arg0), (int)(req.arg1));
        break;
    case tm5_custom::SetEventRequest::STOP:
        rb = iface_.set_stop();
        break;
    case tm5_custom::SetEventRequest::PAUSE:
        rb = iface_.set_pause();
        break;
    case tm5_custom::SetEventRequest::RESUME:
        rb = iface_.set_resume();
        break;
    }
    res.ok = rb;
    return rb;
}

bool TmRosNode::set_io(tm5_custom::SetIORequest &req, tm5_custom::SetIOResponse &res)
{
    bool rb = iface_.set_io(TmIOModule(req.module), TmIOType(req.type), int(req.pin), req.state);
    res.ok = rb;
    return rb;
}

bool TmRosNode::set_positions(tm5_custom::SetPositionsRequest &req, tm5_custom::SetPositionsResponse &res)
{
    bool rb = false;
    switch(req.motion_type) {
    case tm5_custom::SetPositionsRequest::PTP_J:
        rb = iface_.set_joint_pos_PTP(req.positions, req.velocity, req.acc_time, req.blend_percentage, req.fine_goal);
        break;
    case tm5_custom::SetPositionsRequest::PTP_T:
        rb = iface_.set_tool_pose_PTP(req.positions, req.velocity, req.acc_time, req.blend_percentage, req.fine_goal);
        break;
    case tm5_custom::SetPositionsRequest::LINE_T:
        rb = iface_.set_tool_pose_Line(req.positions, req.velocity, req.acc_time, req.blend_percentage, req.fine_goal);
        break;
    }
    res.ok = rb;
    return rb;
}
bool TmRosNode::ask_sta(tm5_custom::AskStaRequest &req, tm5_custom::AskStaResponse &res)
{
    res.ok = listenNodeConnection->ask_sta_struct(req.subcmd, req.subdata, req.wait_time, res.subcmd, res.subdata);
    return res.ok;
}

