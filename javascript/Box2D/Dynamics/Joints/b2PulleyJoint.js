var b2PulleyJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PulleyJoint.prototype, b2Joint.prototype)
b2PulleyJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2PulleyJoint.prototype.__constructor = function (def) {
		
		
		this._super(def);
		
		var tMat;
		var tX;
		var tY;
		
		this.m_ground = this.m_bodyA.m_world.m_groundBody;
		
		this.m_groundAnchor1.x = def.groundAnchorA.x - this.m_ground.m_xf.position.x;
		this.m_groundAnchor1.y = def.groundAnchorA.y - this.m_ground.m_xf.position.y;
		
		this.m_groundAnchor2.x = def.groundAnchorB.x - this.m_ground.m_xf.position.x;
		this.m_groundAnchor2.y = def.groundAnchorB.y - this.m_ground.m_xf.position.y;
		
		this.m_localAnchor1.SetV(def.localAnchorA);
		
		this.m_localAnchor2.SetV(def.localAnchorB);
		
		
		this.m_ratio = def.ratio;
		
		this.m_constant = def.lengthA + this.m_ratio * def.lengthB;
		
		this.m_maxLength1 = b2Math.Min(def.maxLengthA, this.m_constant - this.m_ratio * this.b2_minPulleyLength);
		this.m_maxLength2 = b2Math.Min(def.maxLengthB, (this.m_constant - this.b2_minPulleyLength) / this.m_ratio);
		
		this.m_impulse = 0.0;
		this.m_limitImpulse1 = 0.0;
		this.m_limitImpulse2 = 0.0;
		
	}
b2PulleyJoint.prototype.__varz = function(){
this.m_groundAnchor1 =  new b2Vec2();
this.m_groundAnchor2 =  new b2Vec2();
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_u1 =  new b2Vec2();
this.m_u2 =  new b2Vec2();
}
// static methods
// static attributes
// methods
b2PulleyJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
b2PulleyJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
b2PulleyJoint.prototype.GetReactionForce = function (inv_dt) {
		
		
		return new b2Vec2(inv_dt * this.m_impulse * this.m_u2.x, inv_dt * this.m_impulse * this.m_u2.y);
	}
b2PulleyJoint.prototype.GetReactionTorque = function (inv_dt) {
		
		return 0.0;
	}
b2PulleyJoint.prototype.GetGroundAnchorA = function () {
		
		var a = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor1);
		return a;
	}
b2PulleyJoint.prototype.GetGroundAnchorB = function () {
		
		var a = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor2);
		return a;
	}
b2PulleyJoint.prototype.GetLength1 = function () {
		var p = this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
		
		var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		
		var dX = p.x - sX;
		var dY = p.y - sY;
		
		return Math.sqrt(dX*dX + dY*dY);
	}
b2PulleyJoint.prototype.GetLength2 = function () {
		var p = this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
		
		var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		
		var dX = p.x - sX;
		var dY = p.y - sY;
		
		return Math.sqrt(dX*dX + dY*dY);
	}
b2PulleyJoint.prototype.GetRatio = function () {
		return this.m_ratio;
	}
// attributes
b2PulleyJoint.prototype.m_ground =  null;
b2PulleyJoint.prototype.m_groundAnchor1 =  new b2Vec2();
b2PulleyJoint.prototype.m_groundAnchor2 =  new b2Vec2();
b2PulleyJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2PulleyJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2PulleyJoint.prototype.m_u1 =  new b2Vec2();
b2PulleyJoint.prototype.m_u2 =  new b2Vec2();
b2PulleyJoint.prototype.m_constant =  null;
b2PulleyJoint.prototype.m_ratio =  null;
b2PulleyJoint.prototype.m_maxLength1 =  null;
b2PulleyJoint.prototype.m_maxLength2 =  null;
b2PulleyJoint.prototype.m_pulleyMass =  null;
b2PulleyJoint.prototype.m_limitMass1 =  null;
b2PulleyJoint.prototype.m_limitMass2 =  null;
b2PulleyJoint.prototype.m_impulse =  null;
b2PulleyJoint.prototype.m_limitImpulse1 =  null;
b2PulleyJoint.prototype.m_limitImpulse2 =  null;
b2PulleyJoint.prototype.m_state =  0;
b2PulleyJoint.prototype.m_limitState1 =  0;
b2PulleyJoint.prototype.m_limitState2 =  0;
b2PulleyJoint.prototype.b2_minPulleyLength =  2.0;