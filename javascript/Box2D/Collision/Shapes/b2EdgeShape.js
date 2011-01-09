var b2EdgeShape = function() {
b2Shape.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2EdgeShape.prototype, b2Shape.prototype)
b2EdgeShape.prototype._super = function(){ b2Shape.prototype.__constructor.apply(this, arguments) }
b2EdgeShape.prototype.__constructor = function (v1, v2) {
		this._super();
		m_type = e_edgeShape;
		
		m_prevEdge = null;
		m_nextEdge = null;
		
		m_v1 = v1;
		m_v2 = v2;
		
		m_direction.Set(m_v2.x - m_v1.x, m_v2.y - m_v1.y);
		m_length = m_direction.Normalize();
		m_normal.Set(m_direction.y, -m_direction.x);
		
		m_coreV1.Set(-b2Settings.b2_toiSlop * (m_normal.x - m_direction.x) + m_v1.x,
		 -b2Settings.b2_toiSlop * (m_normal.y - m_direction.y) + m_v1.y)
		m_coreV2.Set(-b2Settings.b2_toiSlop * (m_normal.x + m_direction.x) + m_v2.x,
		 -b2Settings.b2_toiSlop * (m_normal.y + m_direction.y) + m_v2.y)
		
		m_cornerDir1 = m_normal;
		m_cornerDir2.Set(-m_normal.x, -m_normal.y);
	}
b2EdgeShape.prototype.__varz = function(){
this.s_supportVec =  new b2Vec2();
}
// static attributes
// static methods
// attributes
b2EdgeShape.prototype.s_supportVec =  new b2Vec2();
// methods
b2EdgeShape.prototype.TestPoint = function (transform, p) {
		return false;
	}
b2EdgeShape.prototype.RayCast = function (output, input, transform) {
		var tMat;
		var rX = input.p2.x - input.p1.x;
		var rY = input.p2.y - input.p1.y;
		
		
		tMat = transform.R;
		var v1X = transform.position.x + (tMat.col1.x * m_v1.x + tMat.col2.x * m_v1.y);
		var v1Y = transform.position.y + (tMat.col1.y * m_v1.x + tMat.col2.y * m_v1.y);
		
		
		var nX = transform.position.y + (tMat.col1.y * m_v2.x + tMat.col2.y * m_v2.y) - v1Y;
		var nY = -(transform.position.x + (tMat.col1.x * m_v2.x + tMat.col2.x * m_v2.y) - v1X);
		
		var k_slop = 100.0 * Number.MIN_VALUE;
		var denom = -(rX * nX + rY * nY);
	
		
		if (denom > k_slop)
		{
			
			var bX = input.p1.x - v1X;
			var bY = input.p1.y - v1Y;
			var a = (bX * nX + bY * nY);
	
			if (0.0 <= a && a <= input.maxFraction * denom)
			{
				var mu2 = -rX * bY + rY * bX;
	
				
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop))
				{
					a /= denom;
					output.fraction = a;
					var nLen = Math.sqrt(nX * nX + nY * nY);
					output.normal.x = nX / nLen;
					output.normal.y = nY / nLen;
					return true;
				}
			}
		}
		
		return false;
	}
b2EdgeShape.prototype.ComputeAABB = function (aabb, transform) {
		var tMat = transform.R;
		
		var v1X = transform.position.x + (tMat.col1.x * m_v1.x + tMat.col2.x * m_v1.y);
		var v1Y = transform.position.y + (tMat.col1.y * m_v1.x + tMat.col2.y * m_v1.y);
		
		var v2X = transform.position.x + (tMat.col1.x * m_v2.x + tMat.col2.x * m_v2.y);
		var v2Y = transform.position.y + (tMat.col1.y * m_v2.x + tMat.col2.y * m_v2.y);
		if (v1X < v2X) {
			aabb.lowerBound.x = v1X;
			aabb.upperBound.x = v2X;
		} else {
			aabb.lowerBound.x = v2X;
			aabb.upperBound.x = v1X;
		}
		if (v1Y < v2Y) {
			aabb.lowerBound.y = v1Y;
			aabb.upperBound.y = v2Y;
		} else {
			aabb.lowerBound.y = v2Y;
			aabb.upperBound.y = v1Y;
		}
	}
b2EdgeShape.prototype.ComputeMass = function (massData, density) {
		massData.mass = 0;
		massData.center.SetV(m_v1);
		massData.I = 0;
	}
b2EdgeShape.prototype.ComputeSubmergedArea = function (
			normal,
			offset,
			xf,
			c) {
		
		
		
		var v0 = new b2Vec2(normal.x * offset, normal.y * offset);
		
		var v1 = b2Math.MulX(xf, m_v1);
		var v2 = b2Math.MulX(xf, m_v2);
		
		var d1 = b2Math.Dot(normal, v1) - offset;
		var d2 = b2Math.Dot(normal, v2) - offset;
		if (d1 > 0)
		{
			if (d2 > 0)
			{
				return 0;
			}
			else
			{
				
				v1.x = -d2 / (d1 - d2) * v1.x + d1 / (d1 - d2) * v2.x;
				v1.y = -d2 / (d1 - d2) * v1.y + d1 / (d1 - d2) * v2.y;
			}
		}
		else
		{
			if (d2 > 0)
			{
				
				v2.x = -d2 / (d1 - d2) * v1.x + d1 / (d1 - d2) * v2.x;
				v2.y = -d2 / (d1 - d2) * v1.y + d1 / (d1 - d2) * v2.y;
			}
			else
			{
				
			}
		}
		
		
		c.x = (v0.x + v1.x + v2.x) / 3;
		c.y = (v0.y + v1.y + v2.y) / 3;
		
		
		
		
		return 0.5 * ( (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x) );
	}
b2EdgeShape.prototype.GetLength = function () {
		return m_length;
	}
b2EdgeShape.prototype.GetVertex1 = function () {
		return m_v1;
	}
b2EdgeShape.prototype.GetVertex2 = function () {
		return m_v2;
	}
b2EdgeShape.prototype.GetCoreVertex1 = function () {
		return m_coreV1;
	}
b2EdgeShape.prototype.GetCoreVertex2 = function () {
		return m_coreV2;
	}
b2EdgeShape.prototype.GetNormalVector = function () {
		return m_normal;
	}
b2EdgeShape.prototype.GetDirectionVector = function () {
		return m_direction;
	}
b2EdgeShape.prototype.GetCorner1Vector = function () {
		return m_cornerDir1;
	}
b2EdgeShape.prototype.GetCorner2Vector = function () {
		return m_cornerDir2;
	}
b2EdgeShape.prototype.Corner1IsConvex = function () {
		return m_cornerConvex1;
	}
b2EdgeShape.prototype.Corner2IsConvex = function () {
		return m_cornerConvex2;
	}
b2EdgeShape.prototype.GetFirstVertex = function (xf) {
		
		var tMat = xf.R;
		return new b2Vec2(xf.position.x + (tMat.col1.x * m_coreV1.x + tMat.col2.x * m_coreV1.y),
		 xf.position.y + (tMat.col1.y * m_coreV1.x + tMat.col2.y * m_coreV1.y));
	}
b2EdgeShape.prototype.GetNextEdge = function () {
		return m_nextEdge;
	}
b2EdgeShape.prototype.GetPrevEdge = function () {
		return m_prevEdge;
	}
b2EdgeShape.prototype.Support = function (xf, dX, dY) {
		var tMat = xf.R;
		
		var v1X = xf.position.x + (tMat.col1.x * m_coreV1.x + tMat.col2.x * m_coreV1.y);
		var v1Y = xf.position.y + (tMat.col1.y * m_coreV1.x + tMat.col2.y * m_coreV1.y);
		
		
		var v2X = xf.position.x + (tMat.col1.x * m_coreV2.x + tMat.col2.x * m_coreV2.y);
		var v2Y = xf.position.y + (tMat.col1.y * m_coreV2.x + tMat.col2.y * m_coreV2.y);
		
		if ((v1X * dX + v1Y * dY) > (v2X * dX + v2Y * dY)) {
			this.s_supportVec.x = v1X;
			this.s_supportVec.y = v1Y;
		} else {
			this.s_supportVec.x = v2X;
			this.s_supportVec.y = v2Y;
		}
		return this.s_supportVec;
	}