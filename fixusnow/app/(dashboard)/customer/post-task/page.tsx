"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { SERVICE_CATEGORIES } from "@/lib/stripe";

export default function PostTaskPage() {
  const router = useRouter();
  const [form, setForm] = useState({
    title: "",
    description: "",
    category: "",
    budget: "",
    address: "",
    scheduledAt: "",
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");

  function update(field: string, value: string) {
    setForm((f) => ({ ...f, [field]: value }));
  }

  async function handleSubmit(e: React.FormEvent) {
    e.preventDefault();
    setLoading(true);
    setError("");

    const res = await fetch("/api/tasks", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        ...form,
        budget: form.budget ? parseFloat(form.budget) : undefined,
        scheduledAt: form.scheduledAt || undefined,
      }),
    });

    if (!res.ok) {
      const data = await res.json();
      setError(data.error || "Failed to create task.");
      setLoading(false);
      return;
    }

    const task = await res.json();
    router.push(`/tasks/${task.id}`);
  }

  return (
    <div className="max-w-2xl mx-auto px-4 py-10">
      <h1 className="text-3xl font-bold text-gray-900 mb-2">Post a Task</h1>
      <p className="text-gray-500 mb-8">Describe what you need done and providers will send you offers.</p>

      {error && (
        <div className="bg-red-50 text-red-600 text-sm p-3 rounded-lg mb-6">{error}</div>
      )}

      <form onSubmit={handleSubmit} className="bg-white rounded-2xl border border-gray-200 p-8 space-y-6">
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">Task Title *</label>
          <input
            type="text"
            required
            minLength={5}
            value={form.title}
            onChange={(e) => update("title", e.target.value)}
            className="w-full border border-gray-300 rounded-lg px-4 py-2.5 focus:outline-none focus:ring-2 focus:ring-blue-500"
            placeholder="e.g. Fix leaking kitchen faucet"
          />
        </div>

        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">Category *</label>
          <select
            required
            value={form.category}
            onChange={(e) => update("category", e.target.value)}
            className="w-full border border-gray-300 rounded-lg px-4 py-2.5 focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            <option value="">Select a category…</option>
            {SERVICE_CATEGORIES.map((cat) => (
              <option key={cat} value={cat}>{cat}</option>
            ))}
          </select>
        </div>

        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">Description *</label>
          <textarea
            required
            minLength={10}
            rows={5}
            value={form.description}
            onChange={(e) => update("description", e.target.value)}
            className="w-full border border-gray-300 rounded-lg px-4 py-2.5 focus:outline-none focus:ring-2 focus:ring-blue-500 resize-none"
            placeholder="Describe the task in detail. What needs to be done? Any special requirements?"
          />
        </div>

        <div className="grid sm:grid-cols-2 gap-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">Budget ($) — optional</label>
            <input
              type="number"
              min="1"
              step="0.01"
              value={form.budget}
              onChange={(e) => update("budget", e.target.value)}
              className="w-full border border-gray-300 rounded-lg px-4 py-2.5 focus:outline-none focus:ring-2 focus:ring-blue-500"
              placeholder="e.g. 200"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">Preferred Date — optional</label>
            <input
              type="datetime-local"
              value={form.scheduledAt}
              onChange={(e) => update("scheduledAt", e.target.value)}
              className="w-full border border-gray-300 rounded-lg px-4 py-2.5 focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
        </div>

        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">Address *</label>
          <input
            type="text"
            required
            minLength={5}
            value={form.address}
            onChange={(e) => update("address", e.target.value)}
            className="w-full border border-gray-300 rounded-lg px-4 py-2.5 focus:outline-none focus:ring-2 focus:ring-blue-500"
            placeholder="123 Main St, Springfield"
          />
        </div>

        <button
          type="submit"
          disabled={loading}
          className="w-full bg-blue-600 text-white py-3 rounded-xl font-semibold hover:bg-blue-700 disabled:opacity-60 transition text-lg"
        >
          {loading ? "Posting…" : "Post Task"}
        </button>
      </form>
    </div>
  );
}
