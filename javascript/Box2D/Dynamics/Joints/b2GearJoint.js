var b2GearJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2GearJoint.prototype, b2Joint.prototype)
b2GearJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2GearJoint.prototype.__constructor = function (def) {
		
		this._super(def);
		
		var type1 = def.joint1.m_type;
		var type2 = def.joint2.m_type;
		
		
		
		
		
		
		this.m_revolute1 = null;
		this.m_prismatic1 = null;
		this.m_revolute2 = null;
		this.m_prismatic2 = null;
		
		var coordinate1;
		var coordinate2;
		
		this.m_ground1 = def.joint1.GetBodyA();
		this.m_bodyA = def.joint1.GetBodyB();
		if (type1 == b2Joint.e_revoluteJoint)
		{
			this.m_revolute1 = def.joint1;
			this.m_groundAnchor1.SetV( this.m_revolute1.m_localAnchor1 );
			this.m_localAnchor1.SetV( this.m_revolute1.m_localAnchor2 );
			coordinate1 = this.m_revolute1.GetJointAngle();
		}
		else
		{
			this.m_prismatic1 = def.joint1;
			this.m_groundAnchor1.SetV( this.m_prismatic1.m_localAnchor1 );
			this.m_localAnchor1.SetV( this.m_prismatic1.m_localAnchor2 );
			coordinate1 = this.m_prismatic1.GetJointTranslation();
		}
		
		this.m_ground2 = def.joint2.GetBodyA();
		this.m_bodyB = def.joint2.GetBodyB();
		if (type2 == b2Joint.e_revoluteJoint)
		{
			this.m_revolute2 = def.joint2;
			this.m_groundAnchor2.SetV( this.m_revolute2.m_localAnchor1 );
			this.m_localAnchor2.SetV( this.m_revolute2.m_localAnchor2 );
			coordinate2 = this.m_revolute2.GetJointAngle();
		}
		else
		{
			this.m_prismatic2 = def.joint2;
			this.m_groundAnchor2.SetV( this.m_prismatic2.m_localAnchor1 );
			this.m_localAnchor2.SetV( this.m_prismatic2.m_localAnchor2 );
			coordinate2 = this.m_prismatic2.GetJointTranslation();
		}
		
		this.m_ratio = def.ratio;
		
		this.m_constant = coordinate1 + this.m_ratio * coordinate2;
		
		this.m_impulse = 0.0;
		
	}
b2GearJoint.prototype.__varz = function(){
this.m_groundAnchor1 =  new b2Vec2();
this.m_groundAnchor2 =  new b2Vec2();
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_J =  new b2Jacobian();
}
// static methods
// static attributes
// methods
b2GearJoint.prototype.GetAnchorA = function () {
		
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
b2GearJoint.prototype.GetAnchorB = function () {
		
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
b2GearJoint.prototype.GetReactionForce = function (inv_dt) {
		
		
		
		return new b2Vec2(inv_dt * this.m_impulse * this.m_J.linearB.x, inv_dt * this.m_impulse * this.m_J.linearB.y);
	}
b2GearJoint.prototype.GetReactionTorque = function (inv_dt) {
		
		
		var tMat = this.m_bodyB.m_xf.R;
		var rX = this.m_localAnchor1.x - this.m_bodyB.m_sweep.localCenter.x;
		var rY = this.m_localAnchor1.y - this.m_bodyB.m_sweep.localCenter.y;
		var tX = tMat.col1.x * rX + tMat.col2.x * rY;
		rY = tMat.col1.y * rX + tMat.col2.y * rY;
		rX = tX;
		
		var PX = this.m_impulse * this.m_J.linearB.x;
		var PY = this.m_impulse * this.m_J.linearB.y;
		
		
		return inv_dt * (this.m_impulse * this.m_J.angularB - rX * PY + rY * PX);
	}
b2GearJoint.prototype.GetRatio = function () {
		return this.m_ratio;
	}
b2GearJoint.prototype.SetRatio = function (ratio) {
		
		this.m_ratio = ratio;
	}
// attributes
b2GearJoint.prototype.m_ground1 =  null;
b2GearJoint.prototype.m_ground2 =  null;
b2GearJoint.prototype.m_revolute1 =  null;
b2GearJoint.prototype.m_prismatic1 =  null;
b2GearJoint.prototype.m_revolute2 =  null;
b2GearJoint.prototype.m_prismatic2 =  null;
b2GearJoint.prototype.m_groundAnchor1 =  new b2Vec2();
b2GearJoint.prototype.m_groundAnchor2 =  new b2Vec2();
b2GearJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2GearJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2GearJoint.prototype.m_J =  new b2Jacobian();
b2GearJoint.prototype.m_constant =  null;
b2GearJoint.prototype.m_ratio =  null;
b2GearJoint.prototype.m_mass =  null;
b2GearJoint.prototype.m_impulse =  null;