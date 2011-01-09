var b2CircleShape = function() {
b2Shape.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2CircleShape.prototype, b2Shape.prototype)
b2CircleShape.prototype._super = function(){ b2Shape.prototype.__constructor.apply(this, arguments) }
b2CircleShape.prototype.__constructor = function (radius ) {
		this._super();
		m_type = e_circleShape;
		m_radius = radius;
	}
b2CircleShape.prototype.__varz = function(){
}
// static attributes
// static methods
// attributes
// methods
b2CircleShape.prototype.Copy = function () {
		var s = new b2CircleShape();
		s.Set(this);
		return s;
	}
b2CircleShape.prototype.Set = function (other) {
		super.Set(other);
		if (other is b2CircleShape)
		{
			var other2 = other;
			m_p.SetV(other2.m_p);
		}
	}
b2CircleShape.prototype.TestPoint = function (transform, p) {
		
		var tMat = transform.R;
		var dX = transform.position.x + (tMat.col1.x * m_p.x + tMat.col2.x * m_p.y);
		var dY = transform.position.y + (tMat.col1.y * m_p.x + tMat.col2.y * m_p.y);
		
		dX = p.x - dX;
		dY = p.y - dY;
		
		return (dX*dX + dY*dY) <= m_radius * m_radius;
	}
b2CircleShape.prototype.RayCast = function (output, input, transform) {
		
		var tMat = transform.R;
		var positionX = transform.position.x + (tMat.col1.x * m_p.x + tMat.col2.x * m_p.y);
		var positionY = transform.position.y + (tMat.col1.y * m_p.x + tMat.col2.y * m_p.y);
		
		
		var sX = input.p1.x - positionX;
		var sY = input.p1.y - positionY;
		
		var b = (sX*sX + sY*sY) - m_radius * m_radius;
		
		
		
		
		
		var rX = input.p2.x - input.p1.x;
		var rY = input.p2.y - input.p1.y;
		
		var c = (sX*rX + sY*rY);
		
		var rr = (rX*rX + rY*rY);
		var sigma = c * c - rr * b;
		
		
		if (sigma < 0.0 || rr < Number.MIN_VALUE)
		{
			return false;
		}
		
		
		var a = -(c + Math.sqrt(sigma));
		
		
		if (0.0 <= a && a <= input.maxFraction * rr)
		{
			a /= rr;
			output.fraction = a;
			
			output.normal.x = sX + a * rX;
			output.normal.y = sY + a * rY;
			output.normal.Normalize();
			return true;
		}
		
		return false;
	}
b2CircleShape.prototype.ComputeAABB = function (aabb, transform) {
		
		var tMat = transform.R;
		var pX = transform.position.x + (tMat.col1.x * m_p.x + tMat.col2.x * m_p.y);
		var pY = transform.position.y + (tMat.col1.y * m_p.x + tMat.col2.y * m_p.y);
		aabb.lowerBound.Set(pX - m_radius, pY - m_radius);
		aabb.upperBound.Set(pX + m_radius, pY + m_radius);
	}
b2CircleShape.prototype.ComputeMass = function (massData, density) {
		massData.mass = density * b2Settings.b2_pi * m_radius * m_radius;
		massData.center.SetV(m_p);
		
		
		
		massData.I = massData.mass * (0.5 * m_radius * m_radius + (m_p.x*m_p.x + m_p.y*m_p.y));
	}
b2CircleShape.prototype.ComputeSubmergedArea = function (
			normal,
			offset,
			xf,
			c) {
		var p = b2Math.MulX(xf, m_p);
		var l = -(b2Math.Dot(normal, p) - offset);
		
		if (l < -m_radius + Number.MIN_VALUE)
		{
			
			return 0;
		}
		if (l > m_radius)
		{
			
			c.SetV(p);
			return Math.PI * m_radius * m_radius;
		}
		
		
		var r2 = m_radius * m_radius;
		var l2 = l * l;
		var area = r2 *( Math.asin(l / m_radius) + Math.PI / 2) + l * Math.sqrt( r2 - l2 );
		var com = -2 / 3 * Math.pow(r2 - l2, 1.5) / area;
		
		c.x = p.x + normal.x * com;
		c.y = p.y + normal.y * com;
		
		return area;
	}
b2CircleShape.prototype.GetLocalPosition = function () {
		return m_p;
	}
b2CircleShape.prototype.SetLocalPosition = function (position) {
		m_p.SetV(position);
	}
b2CircleShape.prototype.GetRadius = function () {
		return m_radius;
	}
b2CircleShape.prototype.SetRadius = function (radius) {
		m_radius = radius;
	}