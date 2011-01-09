var b2RevoluteJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2RevoluteJoint.prototype, b2Joint.prototype)
b2RevoluteJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2RevoluteJoint.prototype.__constructor = function (def) {
		this._super(def);
		
		
		this.m_localAnchor1.SetV(def.localAnchorA);
		
		this.m_localAnchor2.SetV(def.localAnchorB);
		
		this.m_referenceAngle = def.referenceAngle;
		
		this.m_impulse.SetZero();
		this.m_motorImpulse = 0.0;
		
		this.m_lowerAngle = def.lowerAngle;
		this.m_upperAngle = def.upperAngle;
		this.m_maxMotorTorque = def.maxMotorTorque;
		this.m_motorSpeed = def.motorSpeed;
		this.m_enableLimit = def.enableLimit;
		this.m_enableMotor = def.enableMotor;
		this.m_limitState = this.e_inactiveLimit;
	}
b2RevoluteJoint.prototype.__varz = function(){
this.K =  new b2Mat22();
this.K1 =  new b2Mat22();
this.K2 =  new b2Mat22();
this.K3 =  new b2Mat22();
this.impulse3 =  new b2Vec3();
this.impulse2 =  new b2Vec2();
this.reduced =  new b2Vec2();
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_impulse =  new b2Vec3();
this.m_mass =  new b2Mat33();
}
// static methods
// static attributes
b2RevoluteJoint.tImpulse =  new b2Vec2();
// methods
b2RevoluteJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
b2RevoluteJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
b2RevoluteJoint.prototype.GetReactionForce = function (inv_dt) {
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	}
b2RevoluteJoint.prototype.GetReactionTorque = function (inv_dt) {
		return inv_dt * this.m_impulse.z;
	}
b2RevoluteJoint.prototype.GetJointAngle = function () {
		
		
		return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle;
	}
b2RevoluteJoint.prototype.GetJointSpeed = function () {
		
		
		return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
	}
b2RevoluteJoint.prototype.IsLimitEnabled = function () {
		return this.m_enableLimit;
	}
b2RevoluteJoint.prototype.EnableLimit = function (flag) {
		this.m_enableLimit = flag;
	}
b2RevoluteJoint.prototype.GetLowerLimit = function () {
		return this.m_lowerAngle;
	}
b2RevoluteJoint.prototype.GetUpperLimit = function () {
		return this.m_upperAngle;
	}
b2RevoluteJoint.prototype.SetLimits = function (lower, upper) {
		
		this.m_lowerAngle = lower;
		this.m_upperAngle = upper;
	}
b2RevoluteJoint.prototype.IsMotorEnabled = function () {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		return this.m_enableMotor;
	}
b2RevoluteJoint.prototype.EnableMotor = function (flag) {
		this.m_enableMotor = flag;
	}
b2RevoluteJoint.prototype.SetMotorSpeed = function (speed) {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed;
	}
b2RevoluteJoint.prototype.GetMotorSpeed = function () {
		return this.m_motorSpeed;
	}
b2RevoluteJoint.prototype.SetMaxMotorTorque = function (torque) {
		this.m_maxMotorTorque = torque;
	}
b2RevoluteJoint.prototype.GetMotorTorque = function () {
		return this.m_maxMotorTorque;
	}
// attributes
b2RevoluteJoint.prototype.K =  new b2Mat22();
b2RevoluteJoint.prototype.K1 =  new b2Mat22();
b2RevoluteJoint.prototype.K2 =  new b2Mat22();
b2RevoluteJoint.prototype.K3 =  new b2Mat22();
b2RevoluteJoint.prototype.impulse3 =  new b2Vec3();
b2RevoluteJoint.prototype.impulse2 =  new b2Vec2();
b2RevoluteJoint.prototype.reduced =  new b2Vec2();
b2RevoluteJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2RevoluteJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2RevoluteJoint.prototype.m_impulse =  new b2Vec3();
b2RevoluteJoint.prototype.m_motorImpulse =  null;
b2RevoluteJoint.prototype.m_mass =  new b2Mat33();
b2RevoluteJoint.prototype.m_motorMass =  null;
b2RevoluteJoint.prototype.m_enableMotor =  null;
b2RevoluteJoint.prototype.m_maxMotorTorque =  null;
b2RevoluteJoint.prototype.m_motorSpeed =  null;
b2RevoluteJoint.prototype.m_enableLimit =  null;
b2RevoluteJoint.prototype.m_referenceAngle =  null;
b2RevoluteJoint.prototype.m_lowerAngle =  null;
b2RevoluteJoint.prototype.m_upperAngle =  null;
b2RevoluteJoint.prototype.m_limitState =  0;