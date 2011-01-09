var b2DistanceJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2DistanceJoint.prototype, b2Joint.prototype)
b2DistanceJoint.prototype._super = function(){ b2Joint.prototype.__constructor.apply(this, arguments) }
b2DistanceJoint.prototype.__constructor = function (def) {
		this._super(def);
		
		var tMat;
		var tX;
		var tY;
		this.m_localAnchor1.SetV(def.localAnchorA);
		this.m_localAnchor2.SetV(def.localAnchorB);
		
		this.m_length = def.length;
		this.m_frequencyHz = def.frequencyHz;
		this.m_dampingRatio = def.dampingRatio;
		this.m_impulse = 0.0;
		this.m_gamma = 0.0;
		this.m_bias = 0.0;
	}
b2DistanceJoint.prototype.__varz = function(){
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_u =  new b2Vec2();
}
// static methods
// static attributes
// methods
b2DistanceJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
b2DistanceJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
b2DistanceJoint.prototype.GetReactionForce = function (inv_dt) {
		
		
		return new b2Vec2(inv_dt * this.m_impulse * this.m_u.x, inv_dt * this.m_impulse * this.m_u.y);
	}
b2DistanceJoint.prototype.GetReactionTorque = function (inv_dt) {
		
		return 0.0;
	}
b2DistanceJoint.prototype.GetLength = function () {
		return this.m_length;
	}
b2DistanceJoint.prototype.SetLength = function (length) {
		this.m_length = length;
	}
b2DistanceJoint.prototype.GetFrequency = function () {
		return this.m_frequencyHz;
	}
b2DistanceJoint.prototype.SetFrequency = function (hz) {
		this.m_frequencyHz = hz;
	}
b2DistanceJoint.prototype.GetDampingRatio = function () {
		return this.m_dampingRatio;
	}
b2DistanceJoint.prototype.SetDampingRatio = function (ratio) {
		this.m_dampingRatio = ratio;
	}
// attributes
b2DistanceJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2DistanceJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2DistanceJoint.prototype.m_u =  new b2Vec2();
b2DistanceJoint.prototype.m_frequencyHz =  null;
b2DistanceJoint.prototype.m_dampingRatio =  null;
b2DistanceJoint.prototype.m_gamma =  null;
b2DistanceJoint.prototype.m_bias =  null;
b2DistanceJoint.prototype.m_impulse =  null;
b2DistanceJoint.prototype.m_mass =  null;
b2DistanceJoint.prototype.m_length =  null;