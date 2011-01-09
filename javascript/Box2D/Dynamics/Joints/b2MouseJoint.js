var b2MouseJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2MouseJoint.prototype, b2Joint.prototype)
b2MouseJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2MouseJoint.prototype.__constructor = function (def) {
		this._super(def);
		
		
		
		
		
		
		this.m_target.SetV(def.target);
		
		var tX = this.m_target.x - this.m_bodyB.m_xf.position.x;
		var tY = this.m_target.y - this.m_bodyB.m_xf.position.y;
		var tMat = this.m_bodyB.m_xf.R;
		this.m_localAnchor.x = (tX * tMat.col1.x + tY * tMat.col1.y);
		this.m_localAnchor.y = (tX * tMat.col2.x + tY * tMat.col2.y);
		
		this.m_maxForce = def.maxForce;
		this.m_impulse.SetZero();
		
		this.m_frequencyHz = def.frequencyHz;
		this.m_dampingRatio = def.dampingRatio;
		
		this.m_beta = 0.0;
		this.m_gamma = 0.0;
	}
b2MouseJoint.prototype.__varz = function(){
this.K =  new b2Mat22();
this.K1 =  new b2Mat22();
this.K2 =  new b2Mat22();
this.m_localAnchor =  new b2Vec2();
this.m_target =  new b2Vec2();
this.m_impulse =  new b2Vec2();
this.m_mass =  new b2Mat22();
this.m_C =  new b2Vec2();
}
// static methods
// static attributes
// methods
b2MouseJoint.prototype.GetAnchorA = function () {
		return this.m_target;
	}
b2MouseJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor);
	}
b2MouseJoint.prototype.GetReactionForce = function (inv_dt) {
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	}
b2MouseJoint.prototype.GetReactionTorque = function (inv_dt) {
		return 0.0;
	}
b2MouseJoint.prototype.GetTarget = function () {
		return this.m_target;
	}
b2MouseJoint.prototype.SetTarget = function (target) {
		if (this.m_bodyB.IsAwake() == false){
			this.m_bodyB.SetAwake(true);
		}
		this.m_target = target;
	}
b2MouseJoint.prototype.GetMaxForce = function () {
		return this.m_maxForce;
	}
b2MouseJoint.prototype.SetMaxForce = function (maxForce) {
		this.m_maxForce = maxForce;
	}
b2MouseJoint.prototype.GetFrequency = function () {
		return this.m_frequencyHz;
	}
b2MouseJoint.prototype.SetFrequency = function (hz) {
		this.m_frequencyHz = hz;
	}
b2MouseJoint.prototype.GetDampingRatio = function () {
		return this.m_dampingRatio;
	}
b2MouseJoint.prototype.SetDampingRatio = function (ratio) {
		this.m_dampingRatio = ratio;
	}
// attributes
b2MouseJoint.prototype.K =  new b2Mat22();
b2MouseJoint.prototype.K1 =  new b2Mat22();
b2MouseJoint.prototype.K2 =  new b2Mat22();
b2MouseJoint.prototype.m_localAnchor =  new b2Vec2();
b2MouseJoint.prototype.m_target =  new b2Vec2();
b2MouseJoint.prototype.m_impulse =  new b2Vec2();
b2MouseJoint.prototype.m_mass =  new b2Mat22();
b2MouseJoint.prototype.m_C =  new b2Vec2();
b2MouseJoint.prototype.m_maxForce =  null;
b2MouseJoint.prototype.m_frequencyHz =  null;
b2MouseJoint.prototype.m_dampingRatio =  null;
b2MouseJoint.prototype.m_beta =  null;
b2MouseJoint.prototype.m_gamma =  null;