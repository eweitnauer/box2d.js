var b2WeldJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2WeldJoint.prototype, b2Joint.prototype)
b2WeldJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2WeldJoint.prototype.__constructor = function (def) {
		this._super(def);
		
		this.m_localAnchorA.SetV(def.localAnchorA);
		this.m_localAnchorB.SetV(def.localAnchorB);
		this.m_referenceAngle = def.referenceAngle;

		this.m_impulse.SetZero();
		this.m_mass = new b2Mat33();
	}
b2WeldJoint.prototype.__varz = function(){
this.m_localAnchorA =  new b2Vec2();
this.m_localAnchorB =  new b2Vec2();
this.m_impulse =  new b2Vec3();
this.m_mass =  new b2Mat33();
}
// static methods
// static attributes
// methods
b2WeldJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	}
b2WeldJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	}
b2WeldJoint.prototype.GetReactionForce = function (inv_dt) {
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	}
b2WeldJoint.prototype.GetReactionTorque = function (inv_dt) {
		return inv_dt * this.m_impulse.z;
	}
// attributes
b2WeldJoint.prototype.m_localAnchorA =  new b2Vec2();
b2WeldJoint.prototype.m_localAnchorB =  new b2Vec2();
b2WeldJoint.prototype.m_referenceAngle =  null;
b2WeldJoint.prototype.m_impulse =  new b2Vec3();
b2WeldJoint.prototype.m_mass =  new b2Mat33();