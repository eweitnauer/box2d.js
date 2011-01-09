var b2Joint = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Joint.prototype.__constructor = function (def) {
		b2Settings.b2Assert(def.bodyA != def.bodyB);
		m_type = def.type;
		m_prev = null;
		m_next = null;
		m_bodyA = def.bodyA;
		m_bodyB = def.bodyB;
		m_collideConnected = def.collideConnected;
		m_islandFlag = false;
		this.m_userData = def.userData;
	}
b2Joint.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
b2Joint.prototype.m_userData =  null;
// methods
b2Joint.prototype.GetType = function () {
		return m_type;
	}
b2Joint.prototype.GetAnchorA = function () {return null}
b2Joint.prototype.GetAnchorB = function () {return null}
b2Joint.prototype.GetReactionForce = function (inv_dt) {return null}
b2Joint.prototype.GetReactionTorque = function (inv_dt) {return 0.0}
b2Joint.prototype.GetBodyA = function () {
		return m_bodyA;
	}
b2Joint.prototype.GetBodyB = function () {
		return m_bodyB;
	}
b2Joint.prototype.GetNext = function () {
		return m_next;
	}
b2Joint.prototype.GetUserData = function () {
		return this.m_userData;
	}
b2Joint.prototype.SetUserData = function (data) {
		this.m_userData = data;
	}
b2Joint.prototype.IsActive = function () {
		return m_bodyA.IsActive() && m_bodyB.IsActive();
	}