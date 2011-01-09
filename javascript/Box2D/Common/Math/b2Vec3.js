var b2Vec3 = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Vec3.prototype.__constructor = function (this.x , this.y , this.z ) {
		this.x = this.x;
		this.y = this.y;
		this.z = this.z;
	}
b2Vec3.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
b2Vec3.prototype.x =  null;
b2Vec3.prototype.y =  null;
b2Vec3.prototype.z =  null;
// methods
b2Vec3.prototype.SetZero = function () {
		this.x = this.y = this.z = 0.0;
	}
b2Vec3.prototype.Set = function (this.x, this.y, this.z) {
		this.x = this.x;
		this.y = this.y;
		this.z = this.z;
	}
b2Vec3.prototype.SetV = function (v) {
		this.x = v.x;
		this.y = v.y;
		this.z = v.z;
	}
b2Vec3.prototype.GetNegative = function () { return new b2Vec3( -this.x, -this.y, -this.z); }
b2Vec3.prototype.NegativeSelf = function () { this.x = -this.x; this.y = -this.y; this.z = -this.z; }
b2Vec3.prototype.Copy = function () {
		return new b2Vec3(this.x,this.y,this.z);
	}
b2Vec3.prototype.Add = function (v) {
		this.x += v.x; this.y += v.y; this.z += v.z;
	}
b2Vec3.prototype.Subtract = function (v) {
		this.x -= v.x; this.y -= v.y; this.z -= v.z;
	}
b2Vec3.prototype.Multiply = function (a) {
		this.x *= a; this.y *= a; this.z *= a;
	}