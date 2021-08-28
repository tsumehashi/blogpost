#ifndef SCENEBODYITEM_H_
#define SCENEBODYITEM_H_

#include <memory>
#include <vector>
#include <functional>
#include <set>
#include <ucnoid/Body>
#include <ucnoid/Link>
#include <ptgl/Core/GraphicsItem.h>

class SceneLinkItem;
using SceneLinkItemPtr = std::shared_ptr<SceneLinkItem>;

class SceneBodyItem;
using SceneBodyItemPtr = std::shared_ptr<SceneBodyItem>;

class SceneLinkItem : public ::ptgl::GraphicsItem {
public:
    SceneLinkItem(SceneBodyItem* graphicsBody, cnoid::Link* link);
    virtual ~SceneLinkItem();

    cnoid::Link* link() const { return link_; }

    void setHoverdColorMode(bool on) { hoverdColorMode_ = on; }

    void setShowShape(bool on) { showShape_ = on; }
    void setShowLink(bool on) { showLink_ = on; }
    void setShowCoM(bool on) { showCoM_ = on; }
    void setShowJointAxis(bool on) { showJointAxis_ = on; }

    bool showShape() const { return showShape_; }
    bool showLink() const { return showLink_; }
    bool showCoM() const { return showCoM_; }
    bool showJointAxis() const { return showJointAxis_; }

protected:
    virtual void renderScene(::ptgl::Renderer3D* r) override;

    // Hover event
    virtual void hoverEnterEvent(::ptgl::GraphicsItemHoverEvent* e) override;
    virtual void hoverLeaveEvent(::ptgl::GraphicsItemHoverEvent* e) override;
    virtual void hoverMoveEvent(::ptgl::GraphicsItemHoverEvent* e) override;

    void initialize();

    void drawLinkShape(::ptgl::Renderer3D* r);
    void drawShapes(::ptgl::Renderer3D* r, cnoid::SgNode* node, const cnoid::Affine3& T);

    SceneBodyItem* sceneBodyItem_ = nullptr;
    cnoid::Link* link_ = nullptr;

    bool hoverd_ = false;
    bool hoverdColorMode_ = true;       // default on

    bool showShape_ = true;
    bool showLink_ = false;
    bool showCoM_ = false;
    bool showJointAxis_ = false;
};

class SceneBodyItem : public ::ptgl::GraphicsItem {
    friend class SceneLinkItem;
public:
    SceneBodyItem(cnoid::BodyPtr body);
    virtual ~SceneBodyItem();

    cnoid::BodyPtr body() const { return body_; }

    void setStartShowLink(const std::string& linkName);

    void setHoverdColorMode(bool on);

    void setShowShape(bool on);
    void setShowLink(bool on);
    void setShowCoM(bool on);
    void setShowJointAxis(bool on);

    bool showShape() const { return showShape_; }
    bool showLink() const { return showLink_; }
    bool showCoM() const { return showCoM_; }
    bool showJointAxis() const { return showJointAxis_; }

    void setOnLinkClickedFunction(std::function<void (std::string)> func) { onLinkClickedFunc_ = func; }
    void setOnLinkHoveredEnterFunction(std::function<void (std::string)> func) { onLinkHoveredEnterFunc_ = func; }
    void setLinkHoveredLeaveFunction(std::function<void (std::string)> func) { onLinkHoveredLeaveFunc_ = func; }

    void setOnPrevProcessFunction(std::function<void (void)> func) { onPrevProcessFunc_ = func; }
    void setOnPostProcessFunction(std::function<void (void)> func) { onPostProcessFunc_ = func; }

protected:
    virtual void renderOverlayScene(::ptgl::Renderer3D* r) override;
    virtual void prevProcess() override;
    virtual void postProcess() override;

    void initialize();

    void drawLink(::ptgl::Renderer3D* r);
    void drawCoM(::ptgl::Renderer3D* r);
    void drawJointAxis(::ptgl::Renderer3D* r);

    cnoid::BodyPtr body_;
    std::set<cnoid::Link*> disableShowLinks_;

    // graphics link item
    std::vector<SceneLinkItemPtr> sceneLinkItems_;

    std::function<void (std::string)> onLinkClickedFunc_;
    std::function<void (std::string)> onLinkHoveredEnterFunc_;
    std::function<void (std::string)> onLinkHoveredLeaveFunc_;
    std::function<void (void)> onPrevProcessFunc_;
    std::function<void (void)> onPostProcessFunc_;

    bool hoverdColorMode_ = true;       // default on

    bool showShape_  = true;
    bool showLink_ = false;
    bool showCoM_ = false;
    bool showJointAxis_ = false;
};

#endif /* SCENEBODYITEM_H_ */
