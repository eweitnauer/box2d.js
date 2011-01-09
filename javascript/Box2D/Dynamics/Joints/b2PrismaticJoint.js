var b2PrismaticJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PrismaticJoint.prototype, b2Joint.prototype)
b2PrismaticJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2PrismaticJoint.prototype.__constructor = function (def) {
		this._super(def);
		
		var tMat;
		var tX;
		var tY;
		
		this.m_localAnchor1.SetV(def.localAnchorA);
		this.m_localAnchor2.SetV(def.localAnchorB);
		this.m_localXAxis1.SetV(def.localAxisA);
		
		
		this.m_localYAxis1.x = -this.m_localXAxis1.y;
		this.m_localYAxis1.y = this.m_localXAxis1.x;
		
		this.m_refAngle = def.referenceAngle;
		
		this.m_impulse.SetZero();
		this.m_motorMass = 0.0;
		this.m_motorImpulse = 0.0;
		
		this.m_lowerTranslation = def.lowerTranslation;
		this.m_upperTranslation = def.upperTranslation;
		this.m_maxMotorForce = def.maxMotorForce;
		this.m_motorSpeed = def.motorSpeed;
		this.m_enableLimit = def.enableLimit;
		this.m_enableMotor = def.enableMotor;
		this.m_limitState = this.e_inactiveLimit;
		
		this.m_axis.SetZero();
		this.m_perp.SetZero();
	}
b2PrismaticJoint.prototype.__varz = function(){
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_localXAxis1 =  new b2Vec2();
this.m_localYAxis1 =  new b2Vec2();
this.m_axis =  new b2Vec2();
this.m_perp =  new b2Vec2();
this.m_K =  new b2Mat33();
this.m_impulse =  new b2Vec3();
}
// static attributes
// static methods
// attributes
b2PrismaticJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2PrismaticJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2PrismaticJoint.prototype.m_localXAxis1 =  new b2Vec2();
b2PrismaticJoint.prototype.m_localYAxis1 =  new b2Vec2();
b2PrismaticJoint.prototype.m_refAngle =  null;
b2PrismaticJoint.prototype.m_axis =  new b2Vec2();
b2PrismaticJoint.prototype.m_perp =  new b2Vec2();
b2PrismaticJoint.prototype.m_s1 =  null;
b2PrismaticJoint.prototype.m_s2 =  null;
b2PrismaticJoint.prototype.m_a1 =  null;
b2PrismaticJoint.prototype.m_a2 =  null;
b2PrismaticJoint.prototype.m_K =  new b2Mat33();
b2PrismaticJoint.prototype.m_impulse =  new b2Vec3();
b2PrismaticJoint.prototype.m_motorMass =  null;
b2PrismaticJoint.prototype.m_motorImpulse =  null;
b2PrismaticJoint.prototype.m_lowerTranslation =  null;
b2PrismaticJoint.prototype.m_upperTranslation =  null;
b2PrismaticJoint.prototype.m_maxMotorForce =  null;
b2PrismaticJoint.prototype.m_motorSpeed =  null;
b2PrismaticJoint.prototype.m_enableLimit =  null;
b2PrismaticJoint.prototype.m_enableMotor =  null;
b2PrismaticJoint.prototype.m_limitState =  0;
// methods
b2PrismaticJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
b2PrismaticJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
b2PrismaticJoint.prototype.GetReactionForce = function (inv_dt) {
		
		return new b2Vec2(	inv_dt * (this.m_impulse.x * this.m_perp.x + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x),
							inv_dt * (this.m_impulse.x * this.m_perp.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y));
	}
b2PrismaticJoint.prototype.GetReactionTorque = function (inv_dt) {
		return inv_dt * this.m_impulse.y;
	}
b2PrismaticJoint.prototype.GetJointTranslation = function () {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		
		var tMat;
		
		var p1 = bA.GetWorldPoint(this.m_localAnchor1);
		var p2 = bB.GetWorldPoint(this.m_localAnchor2);
		
		var dX = p2.x - p1.x;
		var dY = p2.y - p1.y;
		
		var axis = bA.GetWorldVector(this.m_localXAxis1);
		
		
		var translation = axis.x*dX + axis.y*dY;
		return translation;
	}
b2PrismaticJoint.prototype.GetJointSpeed = function () {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		
		var tMat;
		
		
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		
		
		var p1X = bA.m_sweep.c.x + r1X;
		var p1Y = bA.m_sweep.c.y + r1Y;
		
		var p2X = bB.m_sweep.c.x + r2X;
		var p2Y = bB.m_sweep.c.y + r2Y;
		
		var dX = p2X - p1X;
		var dY = p2Y - p1Y;
		
		var axis = bA.GetWorldVector(this.m_localXAxis1);
		
		var v1 = bA.m_linearVelocity;
		var v2 = bB.m_linearVelocity;
		var w1 = bA.m_angularVelocity;
		var w2 = bB.m_angularVelocity;
		
		
		
		
		var speed = (dX*(-w1 * axis.y) + dY*(w1 * axis.x)) + (axis.x * ((( v2.x + (-w2 * r2Y)) - v1.x) - (-w1 * r1Y)) + axis.y * ((( v2.y + (w2 * r2X)) - v1.y) - (w1 * r1X)));
		
		return speed;
	}
b2PrismaticJoint.prototype.IsLimitEnabled = function () {
		return this.m_enableLimit;
	}
b2PrismaticJoint.prototype.EnableLimit = function (flag) {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_enableLimit = flag;
	}
b2PrismaticJoint.prototype.GetLowerLimit = function () {
		return this.m_lowerTranslation;
	}
b2PrismaticJoint.prototype.GetUpperLimit = function () {
		return this.m_upperTranslation;
	}
b2PrismaticJoint.prototype.SetLimits = function (lower, upper) {
		
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_lowerTranslation = lower;
		this.m_upperTranslation = upper;
	}
b2PrismaticJoint.prototype.IsMotorEnabled = function () {
		return this.m_enableMotor;
	}
b2PrismaticJoint.prototype.EnableMotor = function (flag) {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_enableMotor = flag;
	}
b2PrismaticJoint.prototype.SetMotorSpeed = function (speed) {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed;
	}
b2PrismaticJoint.prototype.GetMotorSpeed = function () {
		return this.m_motorSpeed;
	}
b2PrismaticJoint.prototype.SetMaxMotorForce = function (force) {
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_maxMotorForce = force;
	}
b2PrismaticJoint.prototype.GetMotorForce = function () {
		return this.m_motorImpulse;
	}