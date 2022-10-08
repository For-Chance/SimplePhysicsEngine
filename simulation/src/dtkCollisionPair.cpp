#include "dtkCollisionPair.h"
#include "dtkFemSimulation.h"

#include <cassert>
#include <algorithm>

namespace dtk {

	ccontact::ccontact(const dtk::dtkPolygonRigidBody& b, size_t idx) {
		indices = { {idx, idx} };
		std::fill(from_a.begin(), from_a.end(), false);
		position = b.local_to_world(b[idx]);
	}

	bool ccontact::operator==(const ccontact& other) const {
		if (from_a == other.from_a && indices == other.indices) {
			return true;
		}
		decltype(from_a) from_a_swap = { {from_a[1], from_a[0]} };
		decltype(indices) indices_swap = { {indices[1], indices[0]} };
		return from_a_swap == other.from_a && indices_swap == other.indices;
	}

	bool ccontact::operator!=(const ccontact& other) const {
		return !(*this == other);
	}

	// ---------------------------------------------------
	// pair

	dtkCollisionPair::dtkCollisionPair(dtk::dtkRigidBody::ptr a, dtk::dtkRigidBody::ptr b, const dtk::dtkDouble2& normal, const dtkCollisionPair::contact_list& contacts)
		: _a(a), _b(b), _normal(normal), _contacts(contacts) {}

	const dtkCollisionPair::contact_list& dtkCollisionPair::get_contacts() const {
		return _contacts;
	}

	const dtk::dtkDouble2& dtkCollisionPair::get_normal() const {
		return _normal;
	}

	void dtkCollisionPair::pre_step(double dt) {
		static const double kAllowedPenetration = 0.01;
		static const double kBiasFactor = 0.2; // 弹性碰撞系数，1.0为完全弹性碰撞
		auto tangent = normal(_normal);
		auto a = _a.lock();
		auto b = _b.lock();
		for (auto& contact : _contacts) {
			auto kn = a->get_inv_mass() + b->get_inv_mass() +
				dot(a->get_inv_inertia() * cross(cross(contact.ra, _normal), contact.ra) +
					b->get_inv_inertia() * cross(cross(contact.rb, _normal), contact.rb), _normal);
			auto kt = a->get_inv_mass() + b->get_inv_mass() +
				dot(a->get_inv_inertia() * cross(cross(contact.ra, tangent), contact.ra) +
					b->get_inv_inertia() * cross(cross(contact.rb, tangent), contact.rb), tangent);

			contact.mass_normal = 1 / kn;
			contact.mass_tangent = 1 / kt;
			contact.bias = -kBiasFactor / dt * std::min(0.0, contact.separation + kAllowedPenetration);
		}
	}

	void dtkCollisionPair::update_impulse() {
		auto tangent = normal(_normal);
		auto a = _a.lock();
		auto b = _b.lock();
		for (auto& contact : _contacts) {
			dtk::dtkDouble2 dv = (b->get_velocity() + cross(b->get_angular_velocity(), contact.rb)) -
				(a->get_velocity() + cross(a->get_angular_velocity(), contact.ra));

			auto vn = dot(dv, _normal);
			auto dpn = (-vn + contact.bias) * contact.mass_normal;
			dpn = std::max(contact.pn + dpn, 0.0) - contact.pn;

			double friction = std::sqrt(a->get_friction() * b->get_friction());
			auto vt = dot(dv, tangent);
			auto dpt = -vt * contact.mass_tangent;
			dpt = std::max(-friction * contact.pn, std::min(friction * contact.pn, contact.pt + dpt)) - contact.pt;

			auto p = dpn * _normal + dpt * tangent;
			a->update_impulse(-p, contact.ra);
			b->update_impulse(p, contact.rb);
			contact.pn += dpn;
			contact.pt += dpt;
		}
	}

	void dtkCollisionPair::update(const dtkCollisionPair& old_arbiter) {
		const auto& old_contacts = old_arbiter._contacts;
		auto tangent = normal(_normal);
		auto a = _a.lock();
		auto b = _b.lock();
		for (auto& new_contact : _contacts) {
			auto old_contact = std::find(old_contacts.begin(), old_contacts.end(), new_contact);
			if (old_contact != old_contacts.end()) {
				new_contact.pn = old_contact->pn;
				new_contact.pt = old_contact->pt;

				auto p = new_contact.pn * _normal + new_contact.pt * tangent;
				a->update_impulse(-p, new_contact.ra);
				b->update_impulse(p, new_contact.rb);
			}
		}
	}

	void dtkCollisionPair::add_contact(const ccontact& contact) {
		_contacts.push_back(contact);
	}

	// ---------------------------------------------------
	// collision detection

		// 找出最小间隙
	static size_t incident_edge(const dtk::dtkDouble2& N, const dtk::dtkPolygonRigidBody& body) {
		size_t idx = SIZE_MAX;
		auto min_dot = dtk::inf;
		// 遍历B物体的边
		for (size_t i = 0; i < body.count(); ++i) {
			// 获得边上的法向量
			auto edge_normal = normal(body.edge(i));
			// 获得法向量在SAT轴上的投影长度
			auto _dot = dot(edge_normal, N);
			// 找出最小投影，即最小间隙
			if (_dot < min_dot) {
				min_dot = _dot; // 最小间隙
				idx = i; // 返回索引
			}
		}
		return idx;
	}

	static size_t clip(dtkCollisionPair::contact_list& contacts_out,
		const dtkCollisionPair::contact_list& contacts_in,
		size_t idx, const dtk::dtkDouble2& v0, const dtk::dtkDouble2& v1) {
		size_t num_out = 0;
		// 得到A物体的边v0_v1的单位向量
		auto N = normalize((v1 - v0));
		// dist0 = v0_B0 X N
		auto dist0 = cross(contacts_in[0].position - v0, N);
		// dist1 = v0_B1 X N
		auto dist1 = cross(contacts_in[1].position - v0, N);
		if (dist0 <= 0) {
			// v0_B0 与 N 共线或顺时针
			contacts_out[num_out++] = contacts_in[0];
		}
		if (dist1 <= 0) {
			// v0_B1 与 N 共线或顺时针
			contacts_out[num_out++] = contacts_in[1];
		}
		if (dist0 * dist1 < 0) { // 一正一负
			auto total_dist = dist0 - dist1;
			auto v = (contacts_in[0].position * -dist1 + contacts_in[1].position * dist0) / total_dist;
			assert(!std::isnan(v.x) && !std::isnan(v.y));
			contacts_out[num_out].position = v;
			contacts_out[num_out].from_a[0] = true;
			contacts_out[num_out].indices[0] = idx;

			++num_out;
		}
		assert(num_out <= 2);
		return num_out;
	}

	dtkCollisionPair::ptr dtkCollisionPair::is_collide_rr(dtk::dtkPolygonRigidBody::ptr& pa, dtk::dtkPolygonRigidBody::ptr& pb, uint32_t& id) {
		auto _pa = &pa;
		auto _pb = &pb;
		size_t ia, ib;
		double sa, sb;
		if ((sa = pa->SAT(ia, *pb)) >= 0) {
			id = MAKE_ID(pa->get_id(), pb->get_id());
			return nullptr;
		}
		if ((sb = pb->SAT(ib, *pa)) >= 0) {
			id = MAKE_ID(pa->get_id(), pb->get_id());
			return nullptr;
		}
		// 当且仅当SAT全为负，则表示相交
		if (sa < sb) {
			// 有序排列
			std::swap(sa, sb);
			std::swap(ia, ib);
			std::swap(_pa, _pb);
		}
		auto& a = **_pa;
		auto& b = **_pb;
		// 获得SAT的轴
		int idx_a = ia;
		auto N = normal(a.edge(ia));
		// 获得最小间隙时的B物体起点索引

		//auto idx = incident_edge(N, b);
		size_t idx = ib;

		// 获得最小间隙时的B物体终点索引
		size_t prev_idx = (idx - 1) % b.count();
		size_t next_idx = (idx + 1) % b.count();
		// 关节列表，暂时认为边 idx-next_idx 是包括在重叠部分中的
		// 下面需要判断B物体其余边是否与A物体重叠

		dtkCollisionPair::contact_list contacts = { {b, prev_idx}, {b, idx},
										{b, next_idx} };
		auto clipped_contacts = contacts;
		// 遍历A物体
		/*
		for (size_t i = 0; i < a.count(); ++i) {
			if (i == ia) { // 非SAT轴的边
				continue;
			}
			// 起点
			auto v0 = a.local_to_world(a[i]);
			// 终点
			auto v1 = a.local_to_world(a[(i + 1) % a.count()]);
			auto num = clip(clipped_contacts, contacts, i, v0, v1);
			if (num < 2) {
				id = MAKE_ID(a.get_id(), b.get_id());
				return nullptr;
			}
			assert(num == 2);
			contacts = clipped_contacts;
		}*/

		auto va = a.local_to_world(a[idx_a]);
		auto arbiter = dtkFactory::make_arbiter(*_pa, *_pb, N);
		for (auto& contact : clipped_contacts) {
			auto sep = dot(contact.position - va, N);
			if (sep <= 0) {
				contact.separation = sep;
				contact.ra = contact.position - a.local_to_world(a.get_centroid());
				contact.rb = contact.position - b.local_to_world(b.get_centroid());
				arbiter->add_contact(contact);
			}
		}
		id = MAKE_ID(a.get_id(), b.get_id());
		return arbiter;
	}

	static size_t nearest_edge(const dtk::dtkDouble2& pos, const dtk::dtkPolygonRigidBody& body) {
		size_t idx = SIZE_MAX;
		auto min_dot = dtk::inf;
		// 遍历B物体的边
		for (size_t i = 0; i < body.count(); ++i) {
			auto edge_normal = normal(body.edge(i));
			auto P = pos - body.local_to_world(body[i]);
			auto _dot = abs(dot(edge_normal, P));
			// 找出最小投影，即最小间隙
			if (_dot < min_dot) {
				min_dot = _dot; // 最小间隙
				idx = i; // 返回索引
			}
		}
		return idx;
	}

	void dtkCollisionPair::do_collision_mr(dtkMesh::ptr& pa, dtk::dtkPolygonRigidBody::ptr& pb) {
		// 当且仅当SAT全为负，则表示相交
		size_t ia, ib;
		double sa, sb;
		if ((sa = pa->shell->SAT(ia, *pb)) >= 0) {
			return;
		}
		if ((sb = pb->SAT(ib, *pa->shell)) >= 0) {
			return;
		}

		// 处理mesh的顶点在刚体内的情况
		int pa_cnt = pa->shell->count();
		for (int i = 0; i < pa_cnt; i++)
		{
			dtkDouble2 va = pa->shell->local_to_world((*pa->shell)[i]);

			size_t idx = nearest_edge(va, *pb);
			dtkDouble2 Nb = normal(pb->edge(idx));
			dtkDouble2 vb = pb->local_to_world((*pb)[idx]);
			dtkDouble2 vb_next = pb->local_to_world((*pb)[(idx + 1) % pb->count()]);
			double sep = dot((va - vb), Nb);

			bool within_edge = dot(va - vb, vb_next - vb) >= 0 && dot(va - vb_next, vb - vb_next) >= 0;

			if (sep <= 0 && within_edge)
			{
				int mesh_index = pa->vertex(i);
				auto velocity_a = pa->points_v_[mesh_index];
				auto pos_a = pa->points_[mesh_index];
				Vector2f vn(Nb.x, Nb.y);
				Vector2f vl(vn[1], -vn[0]);
				Vector2f va_n = velocity_a.dot(vn) * vn, va_l = velocity_a.dot(vl) * vl;
				va_n *= -0.1;
				velocity_a = va_n + va_l;

				pa->points_v_[mesh_index] = velocity_a;
				pa->points_[mesh_index] = pos_a - sep * vn;
			}
		}

		// 处理刚体的顶点在mesh内的情况
		int pb_cnt = pb->count();
		for (int i = 0; i < pb_cnt; i++)
		{
			dtkDouble2 vb = pb->local_to_world((*pb)[i]);

			size_t idx = nearest_edge(vb, *(pa->shell));
			dtkDouble2 Na = normal(pa->shell->edge(idx));
			dtkDouble2 va = pa->shell->local_to_world((*pa->shell)[idx]);
			dtkDouble2 va_next = pa->shell->local_to_world((*pa->shell)[(idx + 1) % pa->shell->count()]);
			double sep = dot((vb - va), Na);

			bool within_edge = dot(vb - va, va_next - va) >= 0 && dot(vb - va_next, va - va_next) >= 0;

			if (sep <= 0 && within_edge)
			{

				int ia2 = (idx + 1) % pa->shell->count();
				int mesh_index1 = pa->vertex(idx), mesh_index2 = pa->vertex(ia2);
				double l1 = length((*pa->shell)[idx] - vb), l2 = length((*pa->shell)[ia2] - vb);
				Vector2f vn(Na.x, Na.y);
				Vector2f v_sum = ((pa->points_v_[mesh_index1] + pa->points_v_[mesh_index2]).dot(vn)) * vn;
				pa->points_v_[mesh_index1] = -(l2 / (l1 + l2)) * v_sum;
				pa->points_v_[mesh_index2] = -(l1 / (l1 + l2)) * v_sum;

				auto B1 = pa->points_[mesh_index1], B2 = pa->points_[mesh_index2];
				auto P = Vector2f(vb.x, vb.y);
				auto B1B2 = B2 - B1;
				Vector2f P1 = (P - B1).dot(B1B2) * B1B2 / pow(B1B2.norm(), 2) + B1;
				pa->points_[mesh_index1] = B1 + (P - P1);
				pa->points_[mesh_index2] = B2 + (P - P1);

			}
		}

	}
}
