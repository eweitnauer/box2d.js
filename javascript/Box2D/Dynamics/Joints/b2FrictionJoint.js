var b2FrictionJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2FrictionJoint.prototype, b2Joint.prototype)
b2FrictionJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2FrictionJoint.prototype.__constructor = function (def) {
		this._super(def);
		
		this.m_localAnchorA.SetV(def.localAnchorA);
		this.m_localAnchorB.SetV(def.localAnchorB);
		
		this.m_linearMass.SetZero();
		this.m_angularMass = 0.0;
		
		this.m_linearImpulse.SetZero();
		this.m_angularImpulse = 0.0;
		
		this.m_maxForce = def.maxForce;
		this.m_maxTorque = def.maxTorque;
	}
b2FrictionJoint.prototype.__varz = function(){
this.m_localAnchorA =  new b2Vec2();
this.m_localAnchorB =  new b2Vec2();
this.m_linearImpulse =  new b2Vec2();
this.m_linearMass =  new b2Mat22();
}
// static attributes
// static methods
// attributes
b2FrictionJoint.prototype.m_localAnchorA =  new b2Vec2();
b2FrictionJoint.prototype.m_localAnchorB =  new b2Vec2();
b2FrictionJoint.prototype.m_linearImpulse =  new b2Vec2();
b2FrictionJoint.prototype.m_angularImpulse =  null;
b2FrictionJoint.prototype.m_maxForce =  null;
b2FrictionJoint.prototype.m_maxTorque =  null;
b2FrictionJoint.prototype.m_linearMass =  new b2Mat22();
b2FrictionJoint.prototype.m_angularMass =  null;
// methods
b2FrictionJoint.prototype.GetAnchorA = function () {
		return m_bodyA.GetWorldPoint(this.m_localAnchorA);
	}
b2FrictionJoint.prototype.GetAnchorB = function () {
		return m_bodyB.GetWorldPoint(this.m_localAnchorB);
	}
b2FrictionJoint.prototype.GetReactionForce = function (inv_dt) {
		return new b2Vec2(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
	}
b2FrictionJoint.prototype.GetReactionTorque = function (inv_dt) {
		
		return inv_dt * this.m_angularImpulse;
	}
b2FrictionJoint.prototype.SetMaxForce = function (force) {
		this.m_maxForce = force;
	}
b2FrictionJoint.prototype.GetMaxForce = function () {
		return this.m_maxForce;
	}
b2FrictionJoint.prototype.SetMaxTorque = function (torque) {
		this.m_maxTorque = torque;
	}
b2FrictionJoint.prototype.GetMaxTorque = function () {
		return this.m_maxTorque;
	}