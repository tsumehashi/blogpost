#include "SceneBodyItem.h"
#include <ucnoid/SceneDrawables>
#include <ptgl/Core/Renderer3D.h>
#include <ptgl/Util/MathUtil.h>

// SceneLinkItem
SceneLinkItem::SceneLinkItem(SceneBodyItem* sceneBodyItem, cnoid::Link* link)
    : sceneBodyItem_(sceneBodyItem), link_(link)
{
    initialize();
}

SceneLinkItem::~SceneLinkItem()
{

}

void SceneLinkItem::initialize()
{

}

// Hover event
void SceneLinkItem::hoverEnterEvent(::ptgl::GraphicsItemHoverEvent* /*e*/)
{
    hoverd_ = true;

    // emit
    if (sceneBodyItem_->onLinkHoveredEnterFunc_) {
        sceneBodyItem_->onLinkHoveredEnterFunc_(this->name());
    }
}

void SceneLinkItem::hoverLeaveEvent(::ptgl::GraphicsItemHoverEvent* /*e*/)
{
    hoverd_ = false;

    // emit
    if (sceneBodyItem_->onLinkHoveredLeaveFunc_) {
        sceneBodyItem_->onLinkHoveredLeaveFunc_(this->name());
    }
}

void SceneLinkItem::hoverMoveEvent(::ptgl::GraphicsItemHoverEvent* /*e*/)
{
    hoverd_ = true;
}

void SceneLinkItem::renderScene(::ptgl::Renderer3D* r)
{
    if (showShape_) {
        drawLinkShape(r);
    }
}

void SceneLinkItem::drawLinkShape(::ptgl::Renderer3D* r)
{
    if (!link_ || !link_->visualShape()) return;

    cnoid::Affine3 T(cnoid::Affine3::Identity());
    T.translation() = link_->p();
    T.linear() = link_->R() * link_->Rs();

    drawShapes(r, link_->visualShape(), T);
}

void SceneLinkItem::drawShapes(::ptgl::Renderer3D* r, cnoid::SgNode* node, const cnoid::Affine3& T)
{
    if (!node) {
        return;
    }

    if (auto transform = dynamic_cast<cnoid::SgPosTransform*>(node)) {
        const cnoid::Affine3 tT = T * transform->T();
        for (int i = 0; i < transform->numChildren(); ++i) {
            auto child = transform->child(i);
            drawShapes(r, child, tT);
        }

    } else if (auto transform = dynamic_cast<cnoid::SgScaleTransform*>(node)) {
        const cnoid::Affine3 tT = T * transform->T();
        for (int i = 0; i < transform->numChildren(); ++i) {
            auto child = transform->child(i);
            drawShapes(r, child, tT);
        }

    } else if (auto shape = dynamic_cast<cnoid::SgShape*>(node)) {
        if (auto mesh = shape->mesh()) {
            // set material
            if (auto material = shape->material()) {
                float alpha = 1.0 - material->transparency();
                cnoid::Vector3f diffuseColor = material->diffuseColor();
                if (hoverd_ && hoverdColorMode_) {
                    diffuseColor << 1, 0.8, 0.5;
                }
                r->setColor(diffuseColor(0), diffuseColor(1), diffuseColor(2), alpha);
            }

            r->pushMatrix();
            r->transform(T);

            // draw shape
            if (mesh->hasVertices() && mesh->hasNormals()) {
                // draw vertex
                auto vertices = mesh->vertices();
                auto normals = mesh->normals();

                if ((int)mesh->normalIndices().size() == (3 * mesh->numTriangles())) {
                    // set unique name
                    std::string vboName = "sl" + std::to_string((uint64_t)mesh);
                    if (!r->isRegisteredVertices(vboName)) {
                        const auto& triangleVertices = mesh->triangleVertices();
                        const auto& normalIndices = mesh->normalIndices();

                        ::ptgl::VertexList vertexList;
                        for (size_t i = 0; i < triangleVertices.size(); ++i) {
                            auto vi = triangleVertices[i];
                            auto ni = normalIndices[i];
                            const auto& v = (*vertices)[vi];
                            const auto& nv = (*normals)[ni];
                            vertexList.push_back(::ptgl::Vertex(v(0), v(1), v(2), nv(0), nv(1), nv(2)));
                        }

                        r->registerVertices(vboName, vertexList, false);
                    }

                    r->drawRegisteredVertices(vboName);
                }
            } else {
                // draw primitive

                static const cnoid::Vector3 p(cnoid::Vector3::Zero());
                static const cnoid::Matrix3 R(cnoid::Matrix3::Identity());

                // draw mesh
                if (mesh->primitiveType() == cnoid::SgMesh::MESH) {
                    // not support
                } else if (mesh->primitiveType() == cnoid::SgMesh::BOX) {
                    const auto& box = mesh->primitive<cnoid::SgMesh::Box>();
                    const double sides[] = {box.size(0), box.size(1), box.size(2)};
                    r->drawBox(p.data(), R.data(), sides);
                } else if (mesh->primitiveType() == cnoid::SgMesh::SPHERE) {
                    const auto& sphere = mesh->primitive<cnoid::SgMesh::Sphere>();
                    r->drawSphere(p.data(), R.data(), sphere.radius);
                } else if (mesh->primitiveType() == cnoid::SgMesh::CYLINDER) {
                    const auto& cylinder = mesh->primitive<cnoid::SgMesh::Cylinder>();
#if 0
                    static const cnoid::Matrix3 cR(::ptgl::rotationX(M_PI/2));
#else
                    static const cnoid::Matrix3 cR(cnoid::Matrix3::Identity());
#endif
                    r->drawCylinder(p.data(), cR.data(), cylinder.height, cylinder.radius);
                } else if (mesh->primitiveType() == cnoid::SgMesh::CONE) {
                    const auto& cone = mesh->primitive<cnoid::SgMesh::Cone>();
                    r->drawCone(p.data(), R.data(), cone.height, cone.radius);
                } else {
                    // not support
                }

            }

            r->popMatrix();
        }

    } else if (auto group = dynamic_cast<cnoid::SgGroup*>(node)) {
        for (int i = 0; i < group->numChildren(); ++i) {
            auto child = group->child(i);
            drawShapes(r, child, T);
        }
    }
}

// SceneBodyItem
SceneBodyItem::SceneBodyItem(cnoid::BodyPtr body) {
    body_ = body;

    initialize();
}

SceneBodyItem::~SceneBodyItem() {

}

void SceneBodyItem::initialize()
{
    // construct SceneLinkItem
    for (int i = 0; i < body_->numLinks(); ++i) {
        SceneLinkItemPtr linkItem = std::make_shared<SceneLinkItem>(this, body_->link(i));
        sceneLinkItems_.push_back(linkItem);
        this->addChild(linkItem);
    }

    setHoverdColorMode(false);
    setShowShape(true);
    setShowLink(false);
    setShowCoM(false);
    setShowJointAxis(false);
}

void SceneBodyItem::setStartShowLink(const std::string& linkName)
{
    disableShowLinks_.clear();
    auto link = body_->link(linkName);
    while (link) {
        disableShowLinks_.emplace(link);
        link = link->parent();
    }
}

void SceneBodyItem::setHoverdColorMode(bool on)
{
    hoverdColorMode_ = on;
    for (auto item : sceneLinkItems_) {
        item->setHoverdColorMode(on);
    }
}

void SceneBodyItem::setShowShape(bool on)
{
    showShape_ = on;
    for (auto item : sceneLinkItems_) {
        item->setShowShape(on);
    }
}

void SceneBodyItem::setShowLink(bool on)
{
    showLink_ = on;
    for (auto item : sceneLinkItems_) {
        item->setShowLink(on);
    }
}

void SceneBodyItem::setShowCoM(bool on)
{
    showCoM_ = on;
    for (auto item : sceneLinkItems_) {
        item->setShowCoM(on);
    }
}

void SceneBodyItem::setShowJointAxis(bool on)
{
    showJointAxis_ = on;
    for (auto item : sceneLinkItems_) {
        item->setShowJointAxis(on);
    }
}

void SceneBodyItem::renderOverlayScene(::ptgl::Renderer3D* r)
{
    if (showLink_) {
        drawLink(r);
    }

    if (showCoM_) {
        drawCoM(r);
    }

    if (showJointAxis_) {
        drawJointAxis(r);
    }
}

void SceneBodyItem::prevProcess()
{
    if (onPrevProcessFunc_) {
        onPrevProcessFunc_();
    }
}

void SceneBodyItem::postProcess()
{
    if (onPostProcessFunc_) {
        onPostProcessFunc_();
    }
}

void SceneBodyItem::drawLink(::ptgl::Renderer3D* r)
{
    for (auto&& linkItem : sceneLinkItems_) {
        if (!linkItem->showLink()) continue;
        auto link = linkItem->link();
        if (disableShowLinks_.count(link)) continue;

        cnoid::Link* parent = link->parent();

        r->setColor(0,0,1,1);
        r->setPointSize(6);
        r->drawPoint(link->p().data());

        r->setColor(0,0,0,1);
        if (parent) {
            r->drawLine(link->p().data(), parent->p().data());
        }
    }
}

void SceneBodyItem::drawCoM(::ptgl::Renderer3D* r)
{
    cnoid::Vector3 cm(body_->calcCenterOfMass());

    r->setColor(1,1,0);
    r->setPointSize(4);
    for (auto&& linkItem : sceneLinkItems_) {
        if (!linkItem->showCoM()) continue;
        auto link = linkItem->link();
        if (disableShowLinks_.count(link)) continue;

        r->drawPoint(link->wc().data());
    }

    r->setColor(1,1,0);
    r->setPointSize(12);
    r->drawPoint(cm.data());
}

void SceneBodyItem::drawJointAxis(::ptgl::Renderer3D* r)
{
    r->setLineWidth(2);

    for (auto&& linkItem : sceneLinkItems_) {
        if (!linkItem->showJointAxis()) continue;
        auto link = linkItem->link();
        if (disableShowLinks_.count(link)) continue;

        if (link->jointType() == cnoid::Link::ROTATIONAL_JOINT) {
            r->setColor(std::abs(link->a()(0)), std::abs(link->a()(2)), std::abs(link->a()(1)));
            cnoid::Vector3 axis = link->R() * link->a();
            axis.normalize();
            const double l = 0.04;
            const cnoid::Vector3 p0 = link->p() + l * axis;
            const cnoid::Vector3 p1 = link->p() - l * axis;
            r->drawLine(p0.data(), p1.data());
        }
    }
}
