var b2FrictionJointDef = function() {
b2JointDef.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2FrictionJointDef.prototype, b2JointDef.prototype)
b2FrictionJointDef.prototype._super = function(){ b2JointDef.prototype.__constructor.apply(this, arguments) }
b2FrictionJointDef.prototype.__constructor = function () {
		this.type = b2Joint.e_frictionJoint;
		this.maxForce = 0.0;
		this.maxTorque = 0.0;
	}
b2FrictionJointDef.prototype.__varz = function(){
this.localAnchorA =  new b2Vec2();
this.localAnchorB =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2FrictionJointDef.prototype.localAnchorA =  new b2Vec2();
b2FrictionJointDef.prototype.localAnchorB =  new b2Vec2();
b2FrictionJointDef.prototype.maxForce =  null;
b2FrictionJointDef.prototype.maxTorque =  null;
// methods
b2FrictionJointDef.prototype.Initialize = function (bA, bB,
								anchor) {
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.SetV( this.bodyA.GetLocalPoint(anchor));
		this.localAnchorB.SetV( this.bodyB.GetLocalPoint(anchor));
	}